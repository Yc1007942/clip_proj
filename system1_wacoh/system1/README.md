# system1
system1 is a ROS 2 project designed for flexible control of industrial 6-axis robot arms and grippers. It features path-following capabilities combined with admittance control that adjusts robot motion in response to external forces.

The architecture consists of two main nodes: MotionController, which receives task commands from higher-level systems, and HardwareInterface, which handles direct hardware communication and executes admittance control calculations.

## Table of Contents
1. [Project Structure](#project-structure)
2. [Prerequisites](#prerequisites)
3. [Setup](#setup)
4. [Build Instructions](#build-instructions)
5. [Running the System](#running-the-system)
6. [Testing](#testing)
7. [Usage Examples](#usage-examples)
8. [Configuration System](#configuration-system)

## Project Structure
```
system1/
├── launch/
│   └── system1.launch.py                         # Launch file to start all nodes
├── resource/
│   └── system1
├── src/
│   └── system1/
│       ├── config/
│       │   ├── __init__.py
│       │   ├── config_loader.py                  # YAML configuration loader
│       │   ├── common.yaml                       # Common configuration
│       │   ├── ur_robot.yaml                     # UR robot configuration
│       │   ├── cobotta_robot.yaml                # COBOTTA robot configuration
│       │   ├── robotiq_gripper.yaml              # Robotiq gripper configuration
│       │   ├── force_sensor.yaml                 # Force sensor configuration
│       │   └── README.md                         # Configuration system documentation
│       ├── hardware_interface/
│       │   ├── __init__.py
│       │   ├── hardware_interface_node.py        # Hardware interface node
│       │   ├── abstract_drivers.py               # Abstract base classes for hardware drivers
│       │   ├── target_pose_corrector.py          # Control correction calculations
│       │   └── drivers/
│       │       ├── __init__.py
│       │       ├── ur_robot_driver.py            # UR robot driver
│       │       ├── cobotta_robot_driver.py       # COBOTTA robot driver
│       │       ├── robotiq_gripper_driver.py     # Robotiq gripper driver
│       │       ├── robotiq_gripper_lib.py        # Robotiq gripper library
│       │       ├── ur_force_sensor_driver.py     # UR built-in force sensor driver
│       │       ├── wacoh_force_sensor_driver.py  # WACOH force sensor driver
│       │       └── wacoh_force_sensor_lib.py     # WACOH force sensor library
│       ├── __init__.py
│       ├── __main__.py                           # Script to launch all nodes in multi-threaded mode
│       └── motion_controller.py                  # Motion controller node
├── tests/                                        # Unit tests using Pytest
│   ├── hardware_interface/
│   │   ├── __init__.py
│   │   ├── conftest.py
│   │   ├── test_hardware_interface_node.py
│   │   ├── test_target_pose_corrector.py
│   │   ├── test_ur_robot_driver.py
│   │   ├── test_cobotta_robot_driver.py
│   │   ├── test_robotiq_gripper_driver.py
│   │   ├── test_ur_force_sensor_driver.py
│   │   └── test_wacoh_force_sensor_driver.py
│   ├── __init__.py
│   └── test_motion_controller.py
├── CMakeLists.txt
├── package.xml
├── pyproject.toml
├── pytest.ini
├── README.md
├── requirements.txt
├── setup.py
└── .gitignore
```

## Prerequisites

### Software

| Item       | Recommended        | Notes                                               |
|-----------|--------------------|----------------------------------------------------|
| OS        | Ubuntu 22.04       | ---                                                |
| Python    | 3.10               | ---                                                |
| ROS 2     | Humble             | ---                                                |

### Hardware
| Item          | Examples                    | Notes                                            |
|---------------|-----------------------------|------------------------------------------------|
| Robot Arm     | UR, COBOTTA PRO            | Other robot arms can be supported by implementing drivers. |
| Gripper       | Robotiq                    | Other grippers can be supported by implementing drivers.     |

## Setup

### Installing Dependencies

```bash
pip install -r requirements.txt
```

### Editing Configuration Files

Edit the YAML configuration files in the `src/system1/config/` directory according to your hardware setup:

1. **common.yaml**: Select the hardware types to use
   - `ROBOT_ARM_TYPE`: "ur" or "cobotta"
   - `GRIPPER_TYPE`: "robotiq"
   - `FORCE_SENSOR_TYPE`: "ur_builtin" or "wacoh"

2. **Robot-specific configuration**:
   - For UR robots: Set the IP address in `ur_robot.yaml`
   - For COBOTTA: Set the PLC connection information in `cobotta_robot.yaml`

3. **Gripper configuration**: Set the IP address and port in `robotiq_gripper.yaml`

**Note**: After modifying YAML files, no package rebuild is required. Simply restart the nodes for the new configuration to take effect.

## Build Instructions
From the workspace root directory, execute the following commands to build:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build
```

### Loading the Environment Setup
After the build completes, load the environment setup with:

```bash
cd ros2_ws
source install/setup.bash
```

## Running the System
You can launch system1 with the following command:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch system1 system1.launch.py
```

### Changing Parameters

To modify runtime parameters, edit the corresponding YAML file in the `src/system1/config/` directory:

1. **Adjusting control parameters** (`common.yaml`)
   - `MOTION_CONTROLLER_RATE_HZ`: Motion controller update rate
   - `HARDWARE_INTERFACE_RATE_HZ`: Hardware interface update rate
   - Various arrival judgment thresholds

2. **Force sensor and admittance control** (`force_sensor.yaml`)
   - Stiffness profiles (stiffness_profiles)
   - Force/torque deadbands
   - Maximum offset amounts

After editing YAML files, simply restart the nodes for changes to take effect (no rebuild required).

## Testing
This project includes unit tests for major functionalities. Test code is located in the `tests/` directory.

### 1. Running Tests
From the workspace root directory, execute:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
cd src/system1
python -m pytest
```

### 2. Test Contents
- **HardwareInterface related**
Located in `tests/hardware_interface/`, includes tests for the hardware interface node, control correction (target_pose_corrector), and various hardware drivers.

- **MotionController tests**
Located in `tests/test_motion_controller.py`, tests various functions of the motion controller.

## Usage Examples

### A. Testing Gripper Operations
To send commands to the gripper (e.g., command 12 to close the gripper):

```bash
source install/setup.bash
ros2 action send_goal /cobotta/robot_action segment_task_pkg/action/ExecuteSegmentTask 'cmd: 12'
```

### B. Testing Path Commands (Waypoint Sequences)
To send path commands to the arm:

```bash
source install/setup.bash
ros2 action send_goal /cobotta/move_hand_with_stiffness segment_task_pkg/action/ExecuteMoveHandWithStiffness 'path:
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: "base_link"
  poses_with_stiffness:
    - pose:
        header:
          stamp:
            sec: 0
            nanosec: 0
          frame_id: "base_link"
        pose:
          position:
            x: 0.5
            y: 0.0
            z: 0.1
          orientation:
            x: 0.7071
            y: -0.7071
            z: 0.0
            w: 0.0
      stiffness_id: 2
    - pose:
        header:
          stamp:
            sec: 0
            nanosec: 0
          frame_id: "base_link"
        pose:
          position:
            x: 0.8
            y: 0.0
            z: 0.1
          orientation:
            x: 0.7071
            y: -0.7071
            z: 0.0
            w: 0.0
      stiffness_id: 2
    - pose:
        header:
          stamp:
            sec: 0
            nanosec: 0
          frame_id: "base_link"
        pose:
          position:
            x: 0.5
            y: 0.0
            z: 0.1
          orientation:
            x: 0.7071
            y: -0.7071
            z: 0.0
            w: 0.0
      stiffness_id: 2'
```

## Configuration System

This project uses a YAML-based configuration system, eliminating the need for rebuilds when changing parameters.

### Configuration File Locations
All configuration files are located in the `src/system1/config/` directory:

- `common.yaml` - Common parameters (control rates, hardware selection, etc.)
- `ur_robot.yaml` - UR robot connection settings
- `cobotta_robot.yaml` - COBOTTA robot PLC settings
- `robotiq_gripper.yaml` - Gripper connection settings
- `force_sensor.yaml` - Force sensor and admittance control parameters

### Procedure for Changing Parameters
1. Open the corresponding YAML file with a text editor
2. Edit and save the parameters
3. Restart the ROS nodes (stop with `Ctrl+C`, then relaunch)

### Key Parameters

#### Hardware Selection (common.yaml)
```yaml
ROBOT_ARM_TYPE: "ur"         # "ur" or "cobotta"
GRIPPER_TYPE: "robotiq"
FORCE_SENSOR_TYPE: "ur_builtin"  # "ur_builtin" or "wacoh"
```

#### Control Parameters (common.yaml)
```yaml
MOTION_CONTROLLER_RATE_HZ: 50.0    # Motion controller rate
HARDWARE_INTERFACE_RATE_HZ: 250.0  # Hardware interface rate
```

#### Robot Connection (ur_robot.yaml / cobotta_robot.yaml)
```yaml
# For UR
ROBOT_IP: "192.168.0.11"

# For COBOTTA
PLC_TARGET_IP: "192.168.0.50"
```

For detailed configuration options, refer to the comments in each YAML file and `src/system1/config/README.md`.
