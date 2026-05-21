import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define package names for description and bringup
    desc_pkg_name = 'platform_description'
    bringup_pkg_name = 'platform_bringup'
    
    # Get absolute paths to the installed packages
    desc_pkg_path = get_package_share_directory(desc_pkg_name)
    bringup_pkg_path = get_package_share_directory(bringup_pkg_name)
    
    # Define paths to URDF (xacro), parameters (yaml), and RViz config
    xacro_file = os.path.join(desc_pkg_path, 'urdf', 'platform.urdf.xacro')
    # NOTE: The parameters file is located in the platform_bringup package
    params_file = os.path.join(bringup_pkg_path, 'config', 'parameters.yaml') 
    rviz_config_file = os.path.join(desc_pkg_path, 'rviz', 'display.rviz')
    
    # Declare a launch argument to optionally use simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Load vehicle parameters from the YAML file
    with open(params_file, 'r') as file:
        vehicle_params = yaml.safe_load(file)['/**']['ros__parameters']
        
    # Process the xacro file into a URDF string, injecting the parameters
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={key: str(value) for key, value in vehicle_params.items()}
    )
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # Node to publish the state of the robot (TF tree) based on the URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )
    
    # Node to display a GUI with sliders for controlling movable joints
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )
    
    # Node to launch RViz for 3D visualization using our config file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    
    # Return the complete launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])