# ROS 2 Drone GNSS/INS Sim (Learning Workspace)

This repository contains a small ROS 2 workspace for practicing:
- ROS 2 Python nodes (`rclpy`) with `turtlesim`
- Robot description setup with URDF/Xacro + RViz
- Basic workspace/package structure for `ament_python` and `ament_cmake`

## Project Structure

```text
ros2-drone-gnss-ins-sim/
├── README.md
├── .gitignore
├── urdf_config2.rviz
└── src/
  ├── my_robot_bringup/             # ament_cmake bringup/simulation package
  │   ├── launch/
  │   │   └── my_robot_gazebo.launch.xml
  │   ├── config/
  │   │   └── gazebo_bridge.yaml
  │   ├── package.xml
  │   └── CMakeLists.txt
    ├── my_robot_controller/          # ament_python package
    │   ├── my_robot_controller/
    │   │   ├── my_first_node.py
    │   │   ├── draw_circle.py
    │   │   ├── draw_triangle.py
    │   │   ├── pose_subscriber.py
    │   │   └── turtle_controller.py
    │   ├── resource/
    │   ├── test/
    │   ├── package.xml
    │   ├── setup.py
    │   └── setup.cfg
    └── my_robot_description/         # ament_cmake package
        ├── launch/
        │   └── display.launch.xml
        ├── rviz/
        │   └── urdf_config.rviz
        ├── urdf/
        │   └── my_robot.urdf.xacro
        ├── package.xml
        └── CMakeLists.txt
```

## What Is Included

### 1) `my_robot_controller`
Python ROS 2 nodes:
- `test_node`: minimal timer logger node
- `draw_circle`: publishes velocity commands for circular motion in `turtlesim`
- `pose_subscriber`: subscribes to `/turtle1/pose`
- `turtle_controller`: basic wall-avoid behavior + pen color service calls
- `draw_triangle`: triangle drawing node scaffold

### 2) `my_robot_description`
Description package with:
- URDF/Xacro model (`urdf/my_robot.urdf.xacro`)
- Gazebo include (`urdf/mobile_base_gazebo.xacro.xml`)
- RViz config (`rviz/urdf_config.rviz`)
- Launch file (`launch/display.launch.xml`) that starts:
  - `robot_state_publisher`
  - `joint_state_publisher_gui`
  - `rviz2`

### 3) `my_robot_bringup`
Bringup package with:
- Gazebo launch orchestration (`launch/my_robot_gazebo.launch.xml`)
- ROS-Gazebo bridge config (`config/gazebo_bridge.yaml`)

## Requirements

- Ubuntu + ROS 2 installed (Humble/Iron/Jazzy style setup)
- `colcon`
- `xacro`, `rviz2`, `robot_state_publisher`, `joint_state_publisher_gui`
- `turtlesim` (for controller nodes)

Example dependencies (adjust for your ROS 2 distro):

```bash
sudo apt update
sudo apt install -y \
  ros-$ROS_DISTRO-turtlesim \
  ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-rviz2 \
  ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-joint-state-publisher-gui \
  python3-colcon-common-extensions
```

## Build

From workspace root:

```bash
cd ros2-drone-gnss-ins-sim
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
source install/setup.bash
```

## Run

### Launch URDF + RViz

```bash
ros2 launch my_robot_description display.launch.xml
```

### Run turtlesim + control nodes

Terminal 1:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2-drone-gnss-ins-sim/install/setup.bash
ros2 run turtlesim turtlesim_node
```

Terminal 2 (choose a node):
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2-drone-gnss-ins-sim/install/setup.bash
ros2 run my_robot_controller draw_circle
```

Other useful nodes:

```bash
ros2 run my_robot_controller pose_subscriber
ros2 run my_robot_controller turtle_controller
ros2 run my_robot_controller test_node
```

## Notes

- The launch file name was standardized to `display.launch.xml`.
- Python cache artifacts are now ignored via `.gitignore`.
- Some package metadata still contains placeholder values (`TODO`) in `package.xml`/`setup.py`.

## Next Improvements (Recommended)

- Replace placeholder package metadata (description, license, maintainer email).
- Fix and verify `draw_triangle.py` behavior and entry point mapping.
- Add one command section for simulation startup scripts.
- Add CI checks for lint + test in GitHub Actions.
