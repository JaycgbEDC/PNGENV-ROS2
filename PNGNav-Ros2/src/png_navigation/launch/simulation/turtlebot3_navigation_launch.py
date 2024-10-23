import os
import re
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav2_yaml = os.path.join(get_package_share_directory('png_navigation'), 'config', 'amcl_config.yaml')

    # Turtlebot3 bringup
    # turtlebot3_bringup_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             os.getenv('AMENT_PREFIX_PATH').split(':')[-1],
    #             'share/turtlebot3_bringup/launch/turtlebot3_state_publisher.launch.py'
    #         )
    #     ]),
    #     launch_arguments={'use_sim_time': 'true'}.items()
    # )

    # Map server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'yaml_filename': os.path.join(get_package_share_directory('png_navigation'), 'map', 'map_gazebo.yaml')}
        ]
    )

    # AMCL
    acml_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
    )

    lcm_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('png_navigation'), 'rviz', 'navigation_static.rviz')],
        output='screen'
    )

    return LaunchDescription([
        # turtlebot3_bringup_node,
        map_server_node,
        acml_node,
        lcm_node,
        rviz_node
    ])