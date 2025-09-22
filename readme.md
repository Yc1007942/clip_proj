
ros2 launch lebai_gripper_ros lebai_gripper.launch.py port:=/dev/ttyUSB0

ros2 topic pub --once --qos-reliability reliable /lebai_gripper/cmd/position std_msgs/UInt8 "data: 50"

ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=false

ros2 topic pub --once --qos-reliability reliable /lebai_gripper/cmd/position std_msgs/UInt8 "data: 50"

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.100 launch_rviz:=false
```
