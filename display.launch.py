import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Get the package share directory and URDF file path
    package_share_dir = get_package_share_directory('test_robot')
    urdf_file = os.path.join(package_share_dir, 'urdf', 'test_robot.urdf')

    # Check if the URDF file exists
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")

    # Parse the URDF file using xacro (if it's a xacro file, otherwise just read the XML file)
    urdf_doc = xacro.parse(open(urdf_file, 'r'))
    xacro.process_doc(urdf_doc)
    robot_description = urdf_doc.toxml()

    # Declare the URDF model argument
    declare_model_arg = DeclareLaunchArgument(
        'model',
        default_value=urdf_file,
        description='Path to the robot model file'
    )

    # Node: Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Node: Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Node: RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        declare_model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz2_node,
    ])

