from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', '-m', 'system1'],
            output='screen',
            shell=False,
        ),
    ])