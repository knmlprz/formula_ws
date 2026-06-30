import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = "jetracer_control"
    pkg_share = get_package_share_directory(pkg)

    # --- robot_description from xacro ---
    robot_description_content = Command(
        [
            "xacro ",
            PathJoinSubstitution(
                [FindPackageShare(pkg), "description", "jetracer.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controllers_yaml = os.path.join(pkg_share, "config", "jetracer_controllers.yaml")

    # --- controller_manager (ros2_control_node) ---
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="both",
    )

    # --- robot_state_publisher (publishes TF from URDF) ---
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # --- spawners ---
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    bicycle_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bicycle_steering_controller", "-c", "/controller_manager"],
    )

    # Start traction/steering controller only after the broadcaster is up
    delay_bicycle = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[bicycle_spawner],
        )
    )

    # --- relay /cmd_vel -> controller reference topic ---
    # bicycle_steering_controller listens on
    #   /bicycle_steering_controller/reference_unstamped  (geometry_msgs/Twist)
    # teleop_twist_keyboard publishes geometry_msgs/Twist on /cmd_vel,
    # so we remap teleop's output directly (see run instructions).
    cmd_vel_relay = Node(
        package="topic_tools",
        executable="relay",
        name="cmd_vel_relay",
        arguments=[
            "/cmd_vel",
<<<<<<< HEAD
            "/bicycle_steering_controller/reference",
=======
            "/bicycle_steering_controller/reference_unstamped",
>>>>>>> jetracer
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            control_node,
            robot_state_pub,
            jsb_spawner,
            delay_bicycle,
            cmd_vel_relay,
        ]
    )
