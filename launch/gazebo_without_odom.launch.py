import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define the package name for your own package
    package_name = 'my_bot'  # Replace with your package name

    # Define the default path to your world file
    world_file_path = os.path.join(
        get_package_share_directory(package_name),
        'worlds', 'trizlabz1.world'
    )

    # Declare the argument for the Gazebo world file with the default value
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file_path,
        description='Path to the Gazebo world file'
    )

    # Include the robot_state_publisher launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp_without_odom.launch.py'
        )),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file and pass the world argument
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # Run the spawner node from the gazebo_ros package
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        declare_world_arg,  # Declare the world argument
        rsp,
        gazebo,
        spawn_entity,
    ])
