from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.timer import Timer

from ..config.config_loader import get_config_loader
from ..hardware_interface.drivers.wacoh_force_sensor_driver import WacohForceSensorDriver


class WacohForceSensorNode(Node):
    """ROS node that publishes wrench data from the Wacoh force/torque sensor."""

    def __init__(self) -> None:
        super().__init__('wacoh_force_sensor_node')

        self._callback_group = ReentrantCallbackGroup()
        self._config_loader = get_config_loader()
        self._sensor_config = self._config_loader.get_sensor_config()
        self._common_config = self._config_loader.get_common_config()

        self._frame_id = self.declare_parameter('frame_id', 'tool0').get_parameter_value().string_value
        publish_rate_hz = self.declare_parameter(
            'publish_rate_hz',
            self._sensor_config.get('wacoh', {}).get('publish_rate_hz', 1500.0),
        ).get_parameter_value().double_value
        self._period = 1.0 / max(publish_rate_hz, 1.0)

        self._driver = WacohForceSensorDriver(logger_callback=self._driver_log)
        self._publisher = self.create_publisher(WrenchStamped, 'wacoh/force_torque', 10)

        self._timer: Optional[Timer] = None
        self._retry_timer: Optional[Timer] = None
        self._connected = False
        self._connect_driver()

    def _driver_log(self, msg: str, level: str = 'info') -> None:
        logger = self.get_logger()
        if level == 'error':
            logger.error(msg)
        elif level == 'warn':
            logger.warning(msg)
        else:
            logger.info(msg)

    def _connect_driver(self) -> None:
        if self._driver.connect():
            self._connected = True
            self.get_logger().info('Wacoh force sensor connected.')
            self._timer = self.create_timer(self._period, self._publish_wrench, callback_group=self._callback_group)
            if self._retry_timer is not None:
                self._retry_timer.cancel()
                self._retry_timer = None
        else:
            self.get_logger().error('Failed to connect Wacoh force sensor; retrying in 5 seconds.')
            if self._retry_timer is None or self._retry_timer.is_canceled():
                self._retry_timer = self.create_timer(5.0, self._retry_connect, callback_group=self._callback_group)

    def _retry_connect(self) -> None:
        if not self._connected:
            self._connect_driver()

    def _publish_wrench(self) -> None:
        if not self._connected:
            return

        wrench = self._driver.get_wrench()
        if wrench is None:
            self.get_logger().warning('No wrench data available.')
            return

        # Ensure we have a numpy array with 6 elements
        if not isinstance(wrench, np.ndarray):
            wrench = np.array(wrench, dtype=float)

        if wrench.shape != (6,):
            self.get_logger().warning(f'Unexpected wrench shape {wrench.shape}')
            return

        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.wrench.force.x = float(wrench[0])
        msg.wrench.force.y = float(wrench[1])
        msg.wrench.force.z = float(wrench[2])
        msg.wrench.torque.x = float(wrench[3])
        msg.wrench.torque.y = float(wrench[4])
        msg.wrench.torque.z = float(wrench[5])

        self._publisher.publish(msg)

    def destroy_node(self) -> bool:
        if self._driver:
            try:
                self._driver.disconnect()
            except Exception as exc:
                self.get_logger().warning(f'Error during sensor disconnect: {exc}')
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = WacohForceSensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutdown requested by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
