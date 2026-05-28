# Driveless Formula Student - Gazebo Bringup Workspace

This ROS 2 workspace (`formula_ws`) contains the core simulation and physical description packages for the Driveless Formula Student vehicle.

## Architecture & Package Structure

The workspace strictly separates the physical definition of the vehicle from its simulation environment, ensuring modularity and reusability.

### 1. `platform_description` (Vehicle DNA)
This package defines "what the robot is". It handles the kinematics, physical properties, and standalone visualization.

*   **`urdf/platform_base.xacro`**: The core structural file. Defines the main chassis (`base_link`), the top platform, wheel axles, and the steering mechanisms for the front wheels.
*   **`urdf/common_properties.xacro`**: The central configuration hub. Contains all parameterized values (dimensions, masses, inertias, offsets) preventing hardcoded numbers in the main `.xacro` file.
*   **`urdf/gazebo_ros2_control.xacro`**: Defines the hardware interfaces for `ros2_control` (e.g., velocity controllers for rear wheels, position controllers for steering joints).
*   **`launch/display.launch.py`**: A lightweight script to spawn the robot model in RViz. It automatically loads Xacro parameters and injects them into the `robot_state_publisher`.
*   **`rviz/display.rviz`**: The pre-configured 3D workspace for RViz, ensuring correct lighting, grids, and TF tree display upon launch.

### 2. `platform_bringup` (Simulation & Control)
This package defines "where the robot lives". It handles the Gazebo physics engine, controllers, and ROS-Gazebo bridges.

*   **`launch/platform_gazebo.launch.py`**: The primary orchestrator. It launches Gazebo, loads the `empty.sdf` world, spawns the URDF model, initiates `ros_gz_bridge`, and activates the Ackermann steering controllers.
*   **`config/parameters.yaml`**: Runtime ROS parameters utilized by the controller nodes (e.g., wheel radius, max steering angles) loaded during simulation.
*   **`config/ros_gz_bridge.yaml`**: The routing table defining which topics (e.g., `/cmd_vel`, `/odom`) are translated between ROS 2 DDS and Gazebo Transport.
*   **`models/`**: Contains environmental 3D assets for the simulation, such as `blue_cone` and `yellow_cone`.

---

## Prerequisites

Ensure you have the following installed on your ROS 2 machine (tested on Humble/Jazzy):
*   **ROS 2** (Desktop Install)
*   **Gazebo Sim** (Harmonic or newer)
*   **ROS 2 Packages:**
    ```bash
    sudo apt install ros-<version>-xacro
    sudo apt install ros-<version>-joint-state-publisher-gui
    sudo apt install ros-<version>-gazebo-ros-pkgs
    sudo apt install ros-<version>-ros-gz-bridge
    ```

---

## Build Instructions

To build the workspace, navigate to the root directory and run:

```bash
colcon build --symlink-install
```

After building, always source the workspace before running commands:
```bash
source install/setup.bash
```

---

## Running the Project

### 1. Visualizing the Model (RViz Only)
Use this command to quickly verify the robot's dimensions, physical structure, and joint movements without launching the heavy Gazebo simulator. Highly recommended after modifying `common_properties.xacro`.

```bash
ros2 launch platform_description display.launch.py
```
*Note: A small GUI window (`joint_state_publisher_gui`) will appear, allowing you to manipulate steering and wheel joints via sliders.*

### 2. Running the Full Simulation (Gazebo)
Use this command to launch the full physics simulation, spawn the robot in an empty world, and load the active controllers.

```bash
ros2 launch platform_bringup platform_gazebo.launch.py
```

---

## Roadmap & Future Work
*   **Sensor Integration:** Mount LiDAR and camera modules on the `platform_link` and configure their Gazebo plugins.
*   **Track Generation:** Create a dynamic `.sdf` world generator utilizing the `blue_cone` and `yellow_cone` models.
*   **Odometry Tuning:** Fine-tune the `ackermann_steering_controller` covariance matrices for realistic slip and friction modeling.