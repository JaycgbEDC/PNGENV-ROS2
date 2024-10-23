from setuptools import setup
from setuptools import find_packages
import os
from glob import glob


package_name = 'png_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/**/*launch.py', recursive=True)),
        (os.path.join('share', package_name, 'config'), glob('share/config/*')),
        (os.path.join('share', package_name, 'map'), glob('share/map/*')),
        (os.path.join('share', package_name, 'model_weights'), glob('share/model_weights/*')),
        (os.path.join('share', package_name, 'rviz'), glob('share/rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kribe',
    maintainer_email='kribe@todo.todo',
    description='The png_navigation package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'irrt_star_node = png_navigation.irrt_star_node:main',
            'nrrt_star_node = png_navigation.nrrt_star_node:main',
            'rrt_star_node = png_navigation.rrt_star_node:main',
            'nirrt_star_node = png_navigation.nirrt_star_node:main',
            'nirrt_star_neural_wrapper_node = png_navigation.nirrt_star_neural_wrapper_node:main',
            'nirrt_star_c_neural_wrapper_node = png_navigation.nirrt_star_c_neural_wrapper_node:main',
            'nrrt_star_neural_wrapper_node = png_navigation.nrrt_star_neural_wrapper_node:main',
            'local_planner_clock = png_navigation.local_planner_clock:main',
            'local_planner_node = png_navigation.local_planner_node:main',
            'global_planner_node = png_navigation.global_planner_node:main',
            'moving_humans_with_noisy_measurements_node = png_navigation.dynamic_obstacles.moving_humans_with_noisy_measurements:main',
            'human_checker_gazebo_node = png_navigation.dynamic_obstacles.human_checker_gazebo:main',
            'local_planner_node_check = png_navigation.dynamic_obstacles.local_planner_node_check:main',
            'global_planner_node_check = png_navigation.dynamic_obstacles.global_planner_node_check:main'
        ],
    },
)
