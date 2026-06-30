# jetracer_control

ROS 2 (Humble) `ros2_control` hardware interface + controller for the
**JetRacer AI Pro** kit on a **Jetson Nano**.

It drives:
- a **steering servo** on PCA9685 channel 0
- an **ESC throttle** on PCA9685 channel 1

via the PCA9685 PWM expander on I2C bus 1 (`/dev/i2c-1`, address `0x40`).
Control is done with the `bicycle_steering_controller`, so you drive it with a
plain `geometry_msgs/Twist` on `/cmd_vel` (keyboard teleop out of the box).

```
keyboard --/cmd_vel--> relay --> bicycle_steering_controller --> JetracerHardware --> PCA9685 (I2C) --> servo + ESC
```

---

## 0. Hardware sanity check first (do this before ROS)

On the Nano:

```bash
sudo apt install -y i2c-tools
i2cdetect -y -r 1
```

You should see a device at `40` (the PCA9685). If it's on a different bus or
address, update the `<param>` values in `description/jetracer.urdf.xacro`.

---

## 1. Docker / workspace setup (Humble on Jetson Nano)

Run a Humble container with I2C access. The key part is passing the I2C device
and running privileged enough to reach it:

```bash
docker run -it --rm \
  --device /dev/i2c-1 \
  --network host \
  ros:humble-ros-base \
  bash
```

(If `--device` isn't enough on your image, `--privileged` works as a fallback.)

Inside the container:

```bash
apt update
apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-bicycle-steering-controller \
  ros-humble-steering-controllers-library \
  ros-humble-robot-state-publisher \
  ros-humble-teleop-twist-keyboard \
  ros-humble-topic-tools \
  ros-humble-xacro \
  libi2c-dev i2c-tools \
  python3-colcon-common-extensions
```

> `libi2c-dev` is required — the hardware interface links against `libi2c`
> for the `i2c_smbus_*` calls.

---

## 2. Build

```bash
mkdir -p ~/ros2_ws/src
cp -r jetracer_control ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select jetracer_control
source install/setup.bash
```

---

## 3. Run

Bring up everything (controller_manager, broadcaster, controller, cmd_vel relay):

```bash
ros2 launch jetracer_control jetracer.launch.py
```

Check it loaded:

```bash
ros2 control list_hardware_interfaces   # should show the joints, "available"
ros2 control list_controllers           # both controllers "active"
```

In a **second terminal** (same container or another one on host network),
drive with the keyboard:

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`teleop_twist_keyboard` publishes `geometry_msgs/Twist` on `/cmd_vel`, which the
launch file relays into `/bicycle_steering_controller/reference_unstamped`.

- `i` / `,` = forward / reverse (linear.x)
- `j` / `l` = steer left / right (angular.z)
- `k` = stop
- Start with a **low speed** (press `z` a few times to lower the speed scale).

---

## 4. CALIBRATION (important — do this carefully)

The default PWM numbers in the URDF are conservative guesses. Tune them in
`description/jetracer.urdf.xacro` under the `<hardware>` block, then rebuild
(or just re-source if you only changed installed files).

### 4a. ESC arming + neutral
Most ESCs must see neutral (~1500 us) at power-on to arm. On `on_configure`
and `on_activate` the interface sends `throttle_neutral_us`. If your car creeps
when "stopped", adjust `throttle_neutral_us` until the wheels are truly still.

### 4b. Throttle scale and limits — **WHEELS OFF THE GROUND FIRST**
Prop the car up so the wheels spin free. Then:
- `throttle_us_per_rad_per_s`: how aggressively velocity maps to PWM. Start
  small (8.0) and increase.
- `throttle_min_us` / `throttle_max_us`: clamp so you can never command a
  dangerous speed. Tighten these (e.g. 1400–1600) while learning.

### 4c. Steering center and direction
- `steer_center_us`: trim until wheels point straight with zero command.
- If the car steers the **wrong way** for a given turn command, flip the sign
  of `steer_us_per_rad` (e.g. `-500` instead of `500`).
- `steer_min_us` / `steer_max_us`: clamp to the servo's safe mechanical range
  so you don't grind the linkage at full lock.

### 4d. Geometry (affects how Twist maps to steering angle)
Measure your actual car and set in BOTH the xacro and the controllers yaml:
- `wheelbase` (front axle → rear axle)
- wheel radius

---

## 5. Common issues

- **"parameter not declared" on controller load** → your Humble patch uses the
  newer parameter names. Edit `config/jetracer_controllers.yaml`: comment out
  `rear_wheels_names`/`front_wheels_names` and use `traction_joints_names`/
  `steering_joints_names` (and `wheel_radius`). The comments in the file show
  exactly what to switch.
- **"Failed to open I2C bus"** → container can't see `/dev/i2c-1`. Re-run docker
  with `--device /dev/i2c-1` (or `--privileged`). Check `ls -l /dev/i2c-*`.
- **Permission denied on I2C** → run the container as root (default) or add the
  user to the `i2c` group.
- **Controller active but nothing moves** → confirm the relay is publishing:
  `ros2 topic echo /bicycle_steering_controller/reference_unstamped` while
  pressing teleop keys. Also `i2cdetect` again to confirm the chip is alive.
- **Car bolts at full speed on launch** → ESC not armed / wrong neutral. Kill
  immediately, fix `throttle_neutral_us`, and keep wheels off the ground while
  tuning.

---

## File layout

```
jetracer_control/
├── CMakeLists.txt
├── package.xml
├── jetracer_control.xml                 # pluginlib hardware plugin export
├── include/jetracer_control/
│   └── jetracer_hardware.hpp
├── src/
│   └── jetracer_hardware.cpp            # SystemInterface + PCA9685 driver
├── description/
│   └── jetracer.urdf.xacro             # robot + <ros2_control> block
├── config/
│   └── jetracer_controllers.yaml       # controller_manager + bicycle ctrl
└── launch/
    └── jetracer.launch.py
```
