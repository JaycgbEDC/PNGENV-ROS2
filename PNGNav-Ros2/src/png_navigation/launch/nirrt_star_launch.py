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

    nirrt_star_node = Node(
        package='png_navigation',
        executable='nirrt_star_node',
        output='screen',
        prefix=['/home/sora/anaconda3/envs/pngenv_ros2/bin/python']
    )
    nirrt_star_neural_wrapper_node = Node(
        package='png_navigation',
        executable='nirrt_star_neural_wrapper_node',
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
            {'use_neural_wrapper': True},
            {'map': LaunchConfiguration('map_name')}
        ]
    )

    return LaunchDescription([
        map_launch_arg, 
        nirrt_star_node,
        nirrt_star_neural_wrapper_node,
        local_planner_clock,
        local_planner_node,
        global_planner_node
    ])