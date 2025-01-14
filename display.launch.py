import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Get the package share directory
    package_share_dir = get_package_share_directory('test_robot_new')
    
    # Point to the URDF file
    urdf_file = os.path.join(package_share_dir, 'urdf', 'test_robot_new.urdf')
    
    # Check if the URDF file exists
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")
    
    # Read the URDF file
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    # Declare the model argument
    declare_model_arg = DeclareLaunchArgument(
        'model',
        default_value=urdf_file,
        description='Path to the robot URDF file'
    )
    
    # Node: Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # Node: Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Node: RViz2 (without loading any config)
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    return LaunchDescription([
        declare_model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz2_node,
    ])
