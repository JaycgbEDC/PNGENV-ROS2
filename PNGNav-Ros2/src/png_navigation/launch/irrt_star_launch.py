from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    map_launch_arg = DeclareLaunchArgument(
        'map_name',
        default_value='map_gazebo',
        description='map fileName'
    )

    irrt_star_node = Node(
        package='png_navigation',
        executable='irrt_star_node',
        output='screen',
        prefix=['/home/sora/anaconda3/envs/pngenv_ros2/bin/python']
    )
    local_planner_clock = Node(
        package='png_navigation',
        executable='local_planner_clock',
        output='screen',
        prefix=['/home/sora/anaconda3/envs/pngenv_ros2/bin/python']
    )
    local_planner_node = Node(
        package='png_navigation',
        executable='local_planner_node',
        output='screen',
        prefix=['/home/sora/anaconda3/envs/pngenv_ros2/bin/python']
    )
    global_planner_node = Node(
        package='png_navigation',
        executable='global_planner_node',
        output='screen',
        prefix=['/home/sora/anaconda3/envs/pngenv_ros2/bin/python'],
        parameters=[
            {'map': LaunchConfiguration('map_name')}
        ]
    )

    return LaunchDescription([
        map_launch_arg,
        irrt_star_node,
        local_planner_clock,
        local_planner_node,
        global_planner_node
    ])