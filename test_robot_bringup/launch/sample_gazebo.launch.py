import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    your_pkg_name_dir = get_package_share_directory("your_pkg_name")  #add your package name here + dir and then again write pkg name

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(your_pkg_name_dir, "urdf", "your_urdf_or_xacro_file.urdf.xacro"),  #add your pkg name and urdf or xacro file
        description="Absolute path to the robot URDF file"
    )
    
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), #make sure there is space after xacro
        value_type=str
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )
    
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(your_pkg_name_dir).parent.resolve())  #add your pkg name + write dir
        ]
    )
    
    # Include Gazebo simulation launch file
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
        launch_arguments=[
            ("gz_args", [" -v 4", " -r ",  "empty.sdf"]   #make sure there is space after r (as shown here)
            )
        ]
    )
    
    # Node: Spawn entity in Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "test_robot_new"]    #add your pkg name
    )
    
    # Return launch description
    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity
    ])
