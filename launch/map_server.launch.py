#!/usr/bin/python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():

    # Package name
    package_name = 'vikings_bot_map_server'

    # Parmaters
    vikings_bot_name_arg = DeclareLaunchArgument("vikings_bot_name",
                default_value="",
                description="Namespace of robot - [vikings_bot_1 or vikings_bot_2]"
    )
    map_file_name_arg = DeclareLaunchArgument('map_file',
                                            default_value='simulation_map.yaml',
                                            description='Map yaml file.')
    use_rviz_arg = DeclareLaunchArgument('use_rviz',
                                            default_value="False",
                                            description='Choose use or not use RVIZ')
    use_sim_arg = DeclareLaunchArgument("use_sim",
                                            default_value="True",
                                            description='Use simulation or real time')

    map_file_name_val = LaunchConfiguration('map_file')
    use_rviz_val = LaunchConfiguration('use_rviz')
    use_sim = LaunchConfiguration("use_sim")


    # RVIZ node
    rviz_config = os.path.join(get_package_share_directory(package_name), 'rviz', 'map_server.rviz')
    rviz_node = Node(package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    condition=IfCondition(PythonExpression([use_rviz_val])),
                    arguments=['-d', rviz_config],
                    parameters=[{'use_sim_time': use_sim}]
                    )
    
    # Map server
    map_file_path = PathJoinSubstitution([get_package_share_directory(package_name), 'maps', map_file_name_val])

    map_server_node = Node(package='nav2_map_server',
                            executable='map_server',
                            name='map_server',
                            namespace=LaunchConfiguration("vikings_bot_name"),
                            output='screen',
                            parameters=[{'use_sim_time': use_sim},
                                        {"topic_name": "map"},
                                        {"frame_id": "map"},
                                        {'yaml_filename': map_file_path}])


    # Lifecycle node
    lifecycle_node = Node(package='nav2_lifecycle_manager',
                            executable='lifecycle_manager',
                            name='map_server_lifecycle_manager',
                            namespace=LaunchConfiguration("vikings_bot_name"),
                            output='screen',
                            parameters=[{'use_sim_time': use_sim},
                                        {'autostart': True},
                                        {'node_names': ['map_server']}])
                    


    return LaunchDescription([  vikings_bot_name_arg,
                                map_file_name_arg,
                                use_rviz_arg,
                                use_sim_arg,
                                LogInfo(msg=map_file_path),
                                rviz_node,
                                map_server_node,
                                lifecycle_node
    ])