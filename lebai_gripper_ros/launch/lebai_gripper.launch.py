from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lebai_gripper_ros',
            executable='lebai_gripper_node',
            name='lebai_gripper',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',   # IMPORTANT: set your serial port here!!
                'slave': 1,
                'baudrate': 115200,
                'publish_rate_hz': 10.0,
            }],
            remappings=[
                ('cmd/position', 'lebai_gripper/cmd/position'),
                ('cmd/force',    'lebai_gripper/cmd/force'),
                ('cmd/speed',    'lebai_gripper/cmd/speed'),
                ('state/position', 'lebai_gripper/state/position'),
                ('state/torque',   'lebai_gripper/state/torque'),
                ('state/done',     'lebai_gripper/state/done'),
                ('state/homed',    'lebai_gripper/state/homed'),
                ('home',           'lebai_gripper/home'),
                ('disable_autohome','lebai_gripper/disable_autohome'),
            ]
        ),
    ])
