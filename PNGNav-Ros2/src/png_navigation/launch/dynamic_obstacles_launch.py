from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moving_humans_with_noisy_measurements_node = Node(
        package='png_navigation',
        executable='moving_humans_with_noisy_measurements_node',
        output='screen',
        prefix=['/home/sora/anaconda3/envs/pngenv_ros2/bin/python']
    )
    human_checker_gazebo_node = Node(
        package='png_navigation',
        executable='human_checker_gazebo_node',
        output='screen',
        prefix=['/home/sora/anaconda3/envs/pngenv_ros2/bin/python']
    )
    return LaunchDescription([
        moving_humans_with_noisy_measurements_node,
        human_checker_gazebo_node
    ])