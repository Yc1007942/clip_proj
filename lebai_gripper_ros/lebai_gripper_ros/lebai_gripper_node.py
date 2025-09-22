"""source install/setup.bash && ros2 launch lebai_gripper_ros lebai_gripper.launch.py port:=/dev/ttyUSB0

    source install/setup.bash && ros2 topic list | grep lebai
"""

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Bool
from std_srvs.srv import Trigger, SetBool
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException

REG_POS_SET      = 0x9C40  # 40000
REG_FORCE_SET    = 0x9C41  # 40001
REG_POS_READ     = 0x9C45  # 40005
REG_TORQUE_READ  = 0x9C46  # 40006
REG_DONE_READ    = 0x9C47  # 40007
REG_HOME_CMD     = 0x9C48  # 40008
REG_NOT_HOMED    = 0x9C49  # 40009 (1=not homed, 0=homed)
REG_SPEED_RW     = 0x9C4A  # 40010
REG_SPEED_SAVE   = 0x9C4B  # 40011
REG_AUTOHOME_CFG = 0x9C9A  # 40090
REG_SET_ADDR     = 0x9C9B  # 40091

class LebaiGripper:
    def __init__(self, port: str, slave: int = 1, baudrate: int = 115200, parity: str = "N", stopbits: int = 1, timeout: float = 1.0):
        self.client = ModbusSerialClient(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            parity=parity,
            stopbits=stopbits,
            timeout=timeout
        )
        self.slave = slave

    def connect(self):
        if not self.client.connect():
            raise RuntimeError("Failed to open serial port/update ur port")

    def close(self):
        try:
            self.client.close()
        except Exception:
            pass

    def _wr(self, addr: int, value: int):
        rq = self.client.write_register(address=addr, value=int(value), device_id=self.slave)
        if rq.isError():
            raise ModbusException(f"Write failed @0x{addr:04X}: {rq}")

    def _rd1(self, addr: int) -> int:
        rr = self.client.read_holding_registers(address=addr, count=1, device_id=self.slave)
        if rr.isError() or not hasattr(rr, "registers"):
            raise ModbusException(f"Read failed @0x{addr:04X}: {rr}")
        return int(rr.registers[0])

    def set_speed(self, percent: int, persist: bool = False):
        percent = max(0, min(100, percent))
        self._wr(REG_SPEED_RW, percent)
        if persist:
            self._wr(REG_SPEED_SAVE, percent)

    def set_force(self, percent: int):
        percent = max(0, min(100, percent))
        self._wr(REG_FORCE_SET, percent)

    def set_position(self, percent: int):
        percent = max(0, min(100, percent))
        self._wr(REG_POS_SET, percent)

    def home(self):
        self._wr(REG_HOME_CMD, 1)

    def disable_auto_home(self, persist: bool = False):
        self._wr(REG_AUTOHOME_CFG, 2 if persist else 1)

    def enable_auto_home_persist(self):
        self._wr(REG_AUTOHOME_CFG, 3)

    def is_homed(self) -> bool:
        return self._rd1(REG_NOT_HOMED) == 0

    def is_done(self) -> bool:
        return self._rd1(REG_DONE_READ) == 1

    def read_position(self) -> int:
        return self._rd1(REG_POS_READ)

    def read_torque(self) -> int:
        return self._rd1(REG_TORQUE_READ)

# ------------- ROS 2 node -------------

class LebaiGripperNode(Node):
    """
    ROS 2 wrapper for Le Bai RS485 Modbus gripper.

    Parameters:
      ~port (string)   : Serial port (e.g., /dev/ttyUSB0, COM3)
      ~slave (int)     : Modbus slave address (default 1)
      ~baudrate (int)  : Baudrate (default 115200)
      ~publish_rate_hz (double): State publish rate (default 10.0)

    Subscriptions:
      /lebai_gripper/cmd/position : std_msgs/UInt8 (0..100)
      /lebai_gripper/cmd/force    : std_msgs/UInt8 (0..100)
      /lebai_gripper/cmd/speed    : std_msgs/UInt8 (0..100)

    Publishers:
      /lebai_gripper/state/position : std_msgs/UInt8
      /lebai_gripper/state/torque   : std_msgs/UInt8
      /lebai_gripper/state/done     : std_msgs/Bool
      /lebai_gripper/state/homed    : std_msgs/Bool

    Services:
      /lebai_gripper/home              : std_srvs/Trigger
      /lebai_gripper/disable_autohome  : std_srvs/SetBool   (data=true -> persist)
    """

    def __init__(self):
        super().__init__("lebai_gripper")

        port = self.declare_parameter("port", "/dev/ttyUSB0").get_parameter_value().string_value
        slave = self.declare_parameter("slave", 1).get_parameter_value().integer_value
        baud  = self.declare_parameter("baudrate", 115200).get_parameter_value().integer_value
        rate  = float(self.declare_parameter("publish_rate_hz", 10.0).get_parameter_value().double_value)

        self.gripper = LebaiGripper(port=port, slave=slave, baudrate=baud)
        self.get_logger().info(f"Connecting to LeBai gripper on {port} @ {baud} (slave={slave})")
        self.gripper.connect()

        # pubs
        self.pub_pos   = self.create_publisher(UInt8, "state/position", 10)
        self.pub_trq   = self.create_publisher(UInt8, "state/torque", 10)
        self.pub_done  = self.create_publisher(Bool,  "state/done", 10)
        self.pub_homed = self.create_publisher(Bool,  "state/homed", 10)

        # subs
        self.sub_pos = self.create_subscription(UInt8, "cmd/position", self.cb_set_position, 10)
        self.sub_force = self.create_subscription(UInt8, "cmd/force", self.cb_set_force, 10)
        self.sub_speed = self.create_subscription(UInt8, "cmd/speed", self.cb_set_speed, 10)

        # srvs
        self.srv_home = self.create_service(Trigger, "home", self.srv_home_cb)
        self.srv_autohome = self.create_service(SetBool, "disable_autohome", self.srv_disable_autohome_cb)

        # timers
        self.timer = self.create_timer(1.0/max(1e-6, rate), self.publish_state)

        self.get_logger().info("LeBai gripper node ready.")

    # --- callbacks ---

    def cb_set_position(self, msg: UInt8):
        try:
            self.gripper.set_position(int(msg.data))
        except Exception as e:
            self.get_logger().error(f"set_position failed: {e}")

    def cb_set_force(self, msg: UInt8):
        try:
            self.gripper.set_force(int(msg.data))
        except Exception as e:
            self.get_logger().error(f"set_force failed: {e}")

    def cb_set_speed(self, msg: UInt8):
        try:
            self.gripper.set_speed(int(msg.data), persist=False)
        except Exception as e:
            self.get_logger().error(f"set_speed failed: {e}")

    def srv_home_cb(self, req, res):
        try:
            self.gripper.home()
            res.success = True
            res.message = "Homing triggered."
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def srv_disable_autohome_cb(self, req, res):
        try:
            self.gripper.disable_auto_home(persist=bool(req.data))
            res.success = True
            res.message = "Auto-homing disabled" + (" (persisted)" if req.data else "")
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def publish_state(self):
        try:
            p = UInt8(data=max(0, min(100, self.gripper.read_position())))
            t = UInt8(data=max(0, min(100, self.gripper.read_torque())))
            d = Bool(data=self.gripper.is_done())
            h = Bool(data=self.gripper.is_homed())
            self.pub_pos.publish(p)
            self.pub_trq.publish(t)
            self.pub_done.publish(d)
            self.pub_homed.publish(h)
        except Exception as e:
            self.get_logger().warn(f"State read failed: {e}")

def main():
    rclpy.init()
    node = LebaiGripperNode()
    try:
        rclpy.spin(node)
    finally:
        node.gripper.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
