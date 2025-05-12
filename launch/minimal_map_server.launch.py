#!/usr/bin/python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():

    # Package name
    package_name = 'vikings_bot_map_server'

    # Parmaters
    map_file_name_arg = DeclareLaunchArgument('map_file',
                                            default_value='warehouse_map.yaml',
                                            description='Map yaml file.')
    map_file_name_val = LaunchConfiguration('map_file')
    


    # Map server
    map_file_path = PathJoinSubstitution([get_package_share_directory(package_name), 'maps', map_file_name_val])

    map_server_node = Node(package='nav2_map_server',
                            executable='map_server',
                            name='map_server',
                            output='screen',
                            parameters=[{'use_sim_time': False},
                                        {"topic_name": "map"},
                                        {"frame_id": "map"},
                                        {'yaml_filename': map_file_path}])


    # Lifecycle node
    lifecycle_node = Node(package='nav2_lifecycle_manager',
                            executable='lifecycle_manager',
                            name='map_server_lifecycle_manager',
                            output='screen',
                            parameters=[{'use_sim_time': False},
                                        {'autostart': True},
                                        {'node_names': ['map_server']}])
                    


    return LaunchDescription([
                                map_file_name_arg,
                                LogInfo(msg=map_file_path),
                                map_server_node,
                                lifecycle_node
    ])