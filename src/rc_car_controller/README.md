# rc_car_controller

Sterownik pojazdu RC (model **Ackermanna**) dla **ROS2 Jazzy**.

Węzeł tłumaczy komendy prędkości (`geometry_msgs/Twist` na `/cmd_vel` oraz
`geometry_msgs/TwistStamped` na `/ackermann_steering_controller/reference`)
na fizyczne sterowanie:
- **układ kierowniczy** — serwo PWM (gpiozero + lgpio),
- **napęd** — dwa silniki przez kontrolery **ODrive** na magistrali **CAN**.

Dodatkowo:
- wyznacza i publikuje **odometrię kołową** na `/odometry/filtered`,
- nadaje **transformację TF** `odom -> base_link` (wymaganą przez SLAM/Nav2),
- realizuje **zatrzymanie awaryjne (E-Stop)** na podstawie `/scan` z LiDAR-u.

## Parametry pojazdu

| Wielkość            | Wartość    | Parametr ROS       |
|---------------------|------------|--------------------|
| Długość całkowita   | 40 cm      | (opis URDF)        |
| Wysokość            | 15 cm      | (opis URDF)        |
| Rozstaw osi         | 34 cm      | `wheelbase = 0.34` |
| Rozstaw kół         | 21 cm      | `track_width = 0.21` |
| Średnica koła       | 62.5 mm    | `wheel_radius = 0.03125` |
| Szerokość koła      | 27 mm      | (opis URDF)        |
| Max kąt skrętu      | 40°        | `steer_limit = 0.698` |

## Struktura paczki

```
rc_car_controller/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── rc_car_controller
├── rc_car_controller/
│   ├── __init__.py
│   └── cmd_vel_controller.py       # główny węzeł
├── config/
│   └── controller_params.yaml      # wszystkie parametry
├── description/
│   ├── urdf/rc_car.urdf.xacro      # opis geometrii
│   └── rviz/rc_car.rviz            # konfiguracja podglądu
└── launch/
    ├── bringup.launch.xml          # start całości
    ├── controller.launch.xml       # sam sterownik
    └── description.launch.xml       # opis robota + RViz
```

## Zależności

ROS: `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `tf2_ros`,
`robot_state_publisher`, `joint_state_publisher`, `xacro`, `rviz2`.

Python (spoza ROS):
```bash
pip3 install python-can gpiozero lgpio
```

Interfejs CAN (przed startem):
```bash
sudo ip link set can0 up type can bitrate 250000
```

## Budowanie

```bash
cd ~/ros2_ws
colcon build --packages-select rc_car_controller
source install/setup.bash
```

## Uruchamianie

Cały stos (opis robota + sterownik):
```bash
ros2 launch rc_car_controller bringup.launch.xml
```

Sam sterownik:
```bash
ros2 launch rc_car_controller controller.launch.xml
```

Wariant testowy bez E-Stopu / na innym interfejsie CAN:
```bash
ros2 launch rc_car_controller controller.launch.xml \
    estop_enabled:=false can_interface:=can1
```

Podgląd w RViz2:
```bash
ros2 launch rc_car_controller description.launch.xml rviz:=true
```

## Sterowanie manualne (test)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Topiki

| Topic | Typ | Kierunek |
|-------|-----|----------|
| `/cmd_vel` | `geometry_msgs/Twist` | sub |
| `/ackermann_steering_controller/reference` | `geometry_msgs/TwistStamped` | sub |
| `/scan` | `sensor_msgs/LaserScan` | sub (E-Stop) |
| `/odometry/filtered` | `nav_msgs/Odometry` | pub |
| `/tf` (`odom -> base_link`) | `tf2_msgs/TFMessage` | pub |

## Uwaga o mapowaniu serwa

Kąt skrętu w radianach jest normalizowany do zakresu serwa `[-1, 1]`
przez podzielenie przez `steer_limit`. Jeżeli kierunek skrętu jest odwrotny,
zamień znak w linii `servo_val` lub przekręć złącze mechanicznie.
