import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Define the package name for your own package
    package_name = 'my_bot'  # Replace with your package name

    # Get the launch directory
    bringup_dir = get_package_share_directory(package_name)

    # Paths to various files
    world_file_path = os.path.join(bringup_dir, 'worlds', 'my_theme.world')
    rviz_config_path = os.path.join(bringup_dir, 'config', 'my_rv.rviz')
    slam_params_file_path = os.path.join(bringup_dir, 'config', 'slam_toolbox_params.yaml')
    nav2_params_file_path = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    bt_xml_file_path = os.path.join(get_package_share_directory('nav2_bt_navigator'), 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    default_params_file = os.path.join(bringup_dir, 'config', 'mapper_params_online_async.yaml')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_world_arg = DeclareLaunchArgument('world', default_value=world_file_path, description='Path to the Gazebo world file')
    declare_use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation/Gazebo clock')
    declare_params_file_arg = DeclareLaunchArgument('params_file', default_value=slam_params_file_path, description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    declare_namespace_arg = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')
    declare_autostart_arg = DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack')
    declare_bt_xml_file_arg = DeclareLaunchArgument('default_bt_xml_filename', default_value=bt_xml_file_path, description='Full path to the behavior tree xml file to use')
    declare_map_subscribe_transient_local_arg = DeclareLaunchArgument('map_subscribe_transient_local', default_value='false', description='Whether to set the map subscriber QoS to transient local')

    # Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # Robot state publisher launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # Spawn entity node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # RViz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-d', rviz_config_path]
    )

    # Async SLAM toolbox node
    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'), 
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'transform_publish_period': 0.1}
        ]
    )

    # Navigation stack nodes
    lifecycle_nodes = ['controller_server', 'planner_server', 'recoveries_server', 'bt_navigator', 'waypoint_follower']
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'default_bt_xml_filename': LaunchConfiguration('default_bt_xml_filename'),
        'autostart': LaunchConfiguration('autostart'),
        'map_subscribe_transient_local': LaunchConfiguration('map_subscribe_transient_local')
    }

    configured_params = RewrittenYaml(
        source_file=nav2_params_file_path,
        root_key=LaunchConfiguration('namespace'),
        param_rewrites=param_substitutions,
        convert_types=True
    )

    nav2_nodes = [
        Node(package='nav2_controller', executable='controller_server', output='screen', parameters=[configured_params], remappings=remappings),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=[configured_params], remappings=remappings),
        Node(package='nav2_recoveries', executable='recoveries_server', name='recoveries_server', output='screen', parameters=[configured_params], remappings=remappings),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', parameters=[configured_params], remappings=remappings),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower', output='screen', parameters=[configured_params], remappings=remappings),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', output='screen', parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}, {'autostart': LaunchConfiguration('autostart')}, {'node_names': lifecycle_nodes}])
    ]

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        declare_world_arg,
        declare_use_sim_time_arg,
        declare_params_file_arg,
        declare_namespace_arg,
        declare_autostart_arg,
        declare_bt_xml_file_arg,
        declare_map_subscribe_transient_local_arg,
        rsp,
        gazebo,
        spawn_entity,
        rviz2_node,
        start_async_slam_toolbox_node
    ] + nav2_nodes)

# Copyright (c) 2018 Intel Corporation
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writi
