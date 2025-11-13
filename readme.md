## Panasonic Clip Project

This project provides workflows for executing gripping tasks on a UR5e robot using MoveIt, dynamic waypoint adjustments, and force/torque feedback. 

---

### 1. Environment

Create and activate the project’s Python environment (if not already done):

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt  # if a requirements file is provided
```

Ensure your ROS 2 environment is sourced. Example (Foxy/Humble, adjust as needed):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash        # From this workspace.
```

---

### 2. Bring-up sequence

1. **UR5e robot driver** (adjust IP to the controller’s address):

    ```bash
    ros2 launch ur_robot_driver ur_control.launch.py launch_rviz:=false
    ```

2. **MoveIt planning stack** (omitting RViz for headless deployments):

    ```bash
    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=false
    ```

3. **Gripper driver** (if the Lebai electric gripper is attached)

    


---

### 3. Running `waypoint_multi.py`

`waypoint_multi.py` performs task selection, dynamic coordinate updates, and force-aware execution cycles.

```bash
source .venv/bin/activate
python waypoint_multi.py --file seti.json --velocity-scale 0.3
```

Key runtime features:

- Prompts to choose a task (e.g. `clip1`, `clip2`).
- Reads XY coordinates from `get_multi_coordinates.py`.
- Maintains a task counter that slightly reduces future approach heights, with optional learning of contact heights.
- Includes commands `test` (simulate a force failure/recovery) and `test-task` (dry-run a task without sending commands).

Abort at any time with `Ctrl+C`. Logs detailing force events, recovery to `failure_home`, and waypoint execution are printed to the console.

---

### 4. Streamlit UI (optional)

The Streamlit interface (referenced in `waypoint_ui.py`) can be launched with:

```bash
streamlit run waypoint_ui.py --server.headless true \
    --server.enableCORS false --server.enableXsrfProtection false
```

This UI mirrors the CLI capabilities, including task selection and live log/status display.

---

### 5. Notes

- `seti.json` contains task definitions (`clip1`, `clip2`, etc.). Ensure the file includes `failure_home` for recovery moves.
- `get_multi_coordinates.py` may run in a test mode (using fixed coordinates); adjust as needed for production.
- When running on real hardware, confirm the UR controller is in **remote mode** and no protective stops are active.

""""
 source /opt/ros/humble/setup.bash && PYTHONPATH=system1_wacoh/system1/src:$PYTHONPATH python3 -m system1.sensor_nodes.wacoh_force_sensor_node
""""
This is for the wacoh sensor
""" source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 topic echo /wacoh/force_torque """