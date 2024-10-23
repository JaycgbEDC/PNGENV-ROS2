# PNGNav-ROS2

This is the ROS2 implementation of NIRRT*-PNG (Neural Informed RRT* with Point-based Network Guidance) for TurtleBot navigation, which is the method in our ICRA 2024 paper

### Neural Informed RRT*: Learning-based Path Planning with Point Cloud State Representations under Admissible Ellipsoidal Constraints

##### [Zhe Huang](https://tedhuang96.github.io/), [Hongyu Chen](https://www.linkedin.com/in/hongyu-chen-91996b22b), [John Pohovey](https://www.linkedin.com/in/johnp14/), [Katherine Driggs-Campbell](https://krdc.web.illinois.edu/)

[[Paper](https://ieeexplore.ieee.org/abstract/document/10611099)] [[arXiv](https://arxiv.org/abs/2309.14595)] [[Main GitHub Repo](https://github.com/tedhuang96/nirrt_star)] [[Robot Demo GitHub Repo](https://github.com/tedhuang96/PNGNav)] [[Project Google Sites](https://sites.google.com/view/nirrt-star)] [[Presentation on YouTube](https://youtu.be/xys6XxMqFqQ)] [[Robot Demo on YouTube](https://youtu.be/XjZqUJ0ufGA)]

All code was developed and tested on Ubuntu 20.04 with CUDA 12.0, ROS foxy, conda 23.11.0, Python 3.8.0, and PyTorch 2.0.1. This repo provides the ROS package `png_navigation` which offers rospy implmentations on RRT*, Informed RRT*, Neural RRT*, and our NIRRT*-PNG for TurtleBot navigation. We offer instructions on how to use `png_navigation` in Gazebo simulation, and `png_navigation` can be readily applied in real world scenarios.

### Citation

If you find this repo useful, please cite

```
@inproceedings{huang2024neural,
  title={Neural Informed RRT*: Learning-based Path Planning with Point Cloud State Representations under Admissible Ellipsoidal Constraints},
  author={Huang, Zhe and Chen, Hongyu and Pohovey, John and Driggs-Campbell, Katherine},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={8742--8748},
  year={2024},
  organization={IEEE}
}
```

## How to Run TurtleBot3 Gazebo Simulation

### Instructions

0. Add the line below to `~/.bashrc`.

```
export TURTLEBOT3_MODEL=waffle_pi
```

1. Launch Gazebo simulation.

```
conda deactivate
source install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

2. Launch `map_server` and `amcl` and `rviz` for Turtlebot3. Note it is the launch file in our `png_navigation` package, which excludes launch of `move_base` and `rviz`.

```
conda deactivate
source install/setup.bash
ros2 launch png_navigation turtlebot3_navigation_launch.py
```

5. Launch the planning algorithm. After you see `Global Planner is initialized.`, you can start planning on rviz by choosing navigation goal.

```
conda deactivate
source install/setup.bash
ros2 launch png_navigation nirrt_star_c_launch.py
```

or any of these lines

```
ros2 launch png_navigation nirrt_star_launch.py
ros2 launch png_navigation nrrt_star_launch.py
ros2 launch png_navigation irrt_star_launch.py
ros2 launch png_navigation rrt_star_launch.py
```

## How to Run Dynamic Obstacles Implementation

Here are the instructions to run the implementation with dynamic obstacles presented in the simulation. We exchange the terms dynamic obstacles and moving humans.

0. Add the line below to `~/.bashrc`.

```
export TURTLEBOT3_MODEL=waffle_pi
```

1. Launch Gazebo simulation.

```
conda deactivate
source install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

2. Launch `map_server` and `amcl` and `rviz`for Turtlebot3. Note it is the launch file in our `png_navigation` package, which excludes launch of `move_base` and `rviz`.

```
conda deactivate
source install/setup.bash
ros2 launch png_navigation turtlebot3_navigation_launch.py
```

3. Create moving humans and human detector.

```
conda deactivate
source install/setup.bash
ros2 launch png_navigation dynamic_obstacles_launch.py
```

7. Start running dynamic obstacle aware planning algorithm.

```
conda deactivate
source install/setup.bash
ros2 launch png_navigation nirrt_star_c_dynamic_obstacles_launch.py
```

or

```
conda deactivate
source install/setup.bash
ros2 launch png_navigation nrrt_star_dynamic_obstacles_launch.py
```

Notes:

- Change moving human speed by changing `vx, vy` in `PNGNav/src/png_navigation/scripts_dynamic_obstacles/moving_humans_with_noisy_measurements.py`.
- change the human detection radius of robot by changing `human_detection_radius` in `PNGNav/src/png_navigation/scripts_dynamic_obstacles/human_checker_gazebo.py`.

## Real World Deployment on yahboomcar with nvidia

## How to Create Your Own Map Yaml File

1. After you finish SLAM and save the map as `.pgm`, you will also get a yaml file. Edit the file which look like this.

```
image: /home/png/map_gazebo.pgm
resolution: 0.010000
origin: [-10.000000, -10.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
setup: 'world'
free_range: [-2, -2, 2, 2]
circle_obstacles: [[1.1, 1.1, 0.15],
                   [1.1, 0, 0.15],
                   [1.1, -1.1, 0.15],
                   [0, 1.1, 0.15],
                   [0, 0, 0.15],
                   [0, -1.1, 0.15],
                   [-1.1, 1.1, 0.15],
                   [-1.1, 0, 0.15],
                   [-1.1, -1.1, 0.15]]
rectangle_obstacles: []
```

The format of fields are as follows.

```
free_range_pixel: [xmin, ymin, xmax, ymax]
circle_obstacles: [[x_center_1, y_center_1, r_1],
                   [x_center_2, y_center_2, r_2],
                   [x_center_3, y_center_3, r_3],
                   ...,
                   [x_center_n, y_center_n, r_n]]
rectangle_obstacles: [[xmin_1, ymin_1, xmax_1, ymax_1],
                      [xmin_2, ymin_2, xmax_2, ymax_2],
                      [xmin_3, ymin_3, xmax_3, ymax_3],
                      ...,
                      [xmin_n, ymin_n, xmax_n, ymax_n]]
```

2. Move `.pgm` and edited `.yaml` files to `PNGNav/src/png_navigation/src/png_navigation/maps`. Keep their names the same, for example `abc.pgm` and `abc.yaml`. When running the launch file, run

```
roslaunch png_navigation nirrt_star_c.launch map:=abc
```

3. You can keep the other fields the same and leave them there. Here is the reference to what these fields mean. We use [ros_map_editor](https://github.com/TheOnceAndFutureSmalltalker/ros_map_editor) to locate the pixels of obstacle keypoints which define the geometry, and then transform from pixel coordinates to world positions. If you are also going to transform from pixel map to get the geometric configurations in the real world, you can use functions `get_transform_pixel_to_world` and `pixel_to_world_coordinates` from `PNGNav/src/png_navigation/src/png_navigation/maps/map_utils.py`.

```
Required fields:

    image : Path to the image file containing the occupancy data; can be absolute, or relative to the location of the YAML file

    resolution : Resolution of the map, meters / pixel

    origin : The 2-D pose of the lower-left pixel in the map, as (x, y, yaw), with yaw as counterclockwise rotation (yaw=0 means no rotation). Many parts of the system currently ignore yaw.

    occupied_thresh : Pixels with occupancy probability greater than this threshold are considered completely occupied.

    free_thresh : Pixels with occupancy probability less than this threshold are considered completely free.

    negate : Whether the white/black free/occupied semantics should be reversed (interpretation of thresholds is unaffected) 
```

## How to Transfer to Your Mobile Robot

1. Modify `robot_config` in `PNGNav/src/png_navigation/src/png_navigation/configs/rrt_star_config.py`. For now, only `robot_config.clearance_radius` matters to global planning.
2. Modify `src/png_navigation/scripts/local_planner_node.py`. In particular, make sure `self.cmd_vel`, `self.odom_frame`, and `self.base_frame` of `class LocalPlanner` match your robot setup. Tune parameters as needed.
