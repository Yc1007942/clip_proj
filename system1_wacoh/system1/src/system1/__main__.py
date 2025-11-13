import sys
import rclpy
from rclpy.executors import MultiThreadedExecutor

from .motion_controller import MotionController
from .hardware_interface.hardware_interface_node import HardwareInterfaceNode

# Load configuration directly from YAML
from .config.config_loader import get_config_loader

# Initialize YAML loader
config_loader = get_config_loader()

# Get driver classes and configuration
driver_info = config_loader.get_driver_classes()
ROBOT_ARM_TYPE = driver_info['types']['robot_arm']
GRIPPER_TYPE = driver_info['types']['gripper']
FORCE_SENSOR_TYPE = driver_info['types']['force_sensor']


def main(args=None):
    rclpy.init(args=args)

    # Logger node for drivers
    temp_logger = rclpy.create_node('temp_logger_for_drivers')

    def logger_callback(msg: str, level: str = 'info'):
        if level == 'info':
            temp_logger.get_logger().info(msg)
        else:
            temp_logger.get_logger().error(msg)

    # Get driver classes
    ArmDriver = driver_info['robot_arm']
    GripDriver = driver_info['gripper']
    SensorDriver = driver_info['force_sensor']

    # Instantiate drivers
    robot_arm = ArmDriver(logger_callback=logger_callback)
    gripper   = GripDriver(logger_callback=logger_callback)

    # Force sensor has different constructor arguments depending on sensor type
    sensor_config = config_loader.get_sensor_config()
    sensor_params = sensor_config.get(FORCE_SENSOR_TYPE, {}).get('sensor_params', {})
    if FORCE_SENSOR_TYPE == 'ur_builtin':
        force_sensor = SensorDriver(
            robot_arm=robot_arm,
            logger_callback=logger_callback,
            **sensor_params
        )
    else:
        force_sensor = SensorDriver(
            logger_callback=logger_callback,
            **sensor_params
        )

    # Connect each driver and check status
    for name, drv, err in [
        ('UR robot arm', robot_arm, 'Failed to connect UR robot arm.'),
        ('Robotiq gripper', gripper, 'Failed to connect Robotiq gripper.'),
        ('force sensor', force_sensor, f'Failed to connect force sensor ({FORCE_SENSOR_TYPE}).'),
    ]:
        if not drv.connect():
            temp_logger.get_logger().error(err)
            sys.exit(1)

    # Create nodes
    motion_node = MotionController()
    hw_node = HardwareInterfaceNode(
        robot_arm      = robot_arm,
        gripper        = gripper,
        force_sensor   = force_sensor,
        # Parameters for admittance control and mount transformation
        adm_params     = sensor_config.get(FORCE_SENSOR_TYPE, {}).get('adm_params'),
        mount_transform= sensor_config.get('mount_transforms', {}).get(FORCE_SENSOR_TYPE),
    )

    # Register with executor and spin
    executor = MultiThreadedExecutor()
    executor.add_node(motion_node)
    executor.add_node(hw_node)

    # Startup log
    sensor_names = {
        'ur_builtin': 'UR built-in force sensor',
        'wacoh':      'Wacoh FM-sensor',
    }
    sn = sensor_names.get(FORCE_SENSOR_TYPE, FORCE_SENSOR_TYPE)
    motion_node.get_logger().info(f'System1 starting up with {sn}')

    try:
        executor.spin()
    except KeyboardInterrupt:
        motion_node.get_logger().info('System1 shutdown requested')
    finally:
        executor.shutdown()
        # Disconnect and destroy nodes
        for drv in (force_sensor, gripper, robot_arm):
            try:
                drv.disconnect()
            except Exception:
                pass
        motion_node.destroy_node()
        hw_node.destroy_node()
        temp_logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()