#!/home/sora/anaconda3/envs/pngenv/bin/python
from os.path import join

import tf_transformations
import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import numpy as np
from PIL import Image

import tf2_ros

from png_navigation.tools.configs.rrt_star_config import Config
from png_navigation.tools.path_planning_classes.rrt_env_2d import Env
from png_navigation.tools.maps.map_utils import get_transform_pixel_to_world, pixel_to_world_coordinates, min_max_aabb

from nav_msgs.msg import Path
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from png_interfaces.srv import SetEnv
from png_interfaces.srv import GetGlobalPlan


def get_env_dict(map_filename, package_path):
    map_folderpath = join(package_path, 'map')
    map_config_filepath = join(map_folderpath, map_filename+'.yaml')
    with open(map_config_filepath, 'r') as file:
        map_config = yaml.safe_load(file)
    if map_config['setup'] != 'world':
        raise NotImplementedError('map_config setup as pixel is not implemented yet.')
    if map_filename=='map_gazebo':
        free_range_world = np.array(map_config['free_range']) # [x1, y1, x2, y2]
        circle_obstacles_world = np.array(map_config['circle_obstacles']) # (n, 3)
        rectangle_obstacles_world = np.array(map_config['rectangle_obstacles']) # (n, 4)
        if len(circle_obstacles_world)==0:
            circle_obstacles_world = None
        if len(rectangle_obstacles_world)==0:
            rectangle_obstacles_world = None
        env_dict = {
            'x_range': (free_range_world[0], free_range_world[2]),
            'y_range': (free_range_world[1], free_range_world[3]),
            'circle_obstacles': circle_obstacles_world,
            'rectangle_obstacles': rectangle_obstacles_world,
        }
    else:
        map_image_filepath = join(map_folderpath, map_filename+'.pgm')
        map_image = Image.open(map_image_filepath)
        A_wp, b_wp, (xp_origin, yp_origin) = get_transform_pixel_to_world(
                map_config,
                map_image,
            )
        free_range_p = np.array(map_config['free_range_pixel']).reshape(-1,2) # (4,) -> (2,2) [[x1,y1],[x2,y2]]
        rectangle_obstacles_p = np.array(map_config['rectangle_obstacles_pixel']).reshape(-1,2) # (n_rect_obs*2,2)
        free_range_w = pixel_to_world_coordinates(free_range_p, A_wp, b_wp).reshape(4) # (x1,y1,x2,y2)
        rectangle_obstacles_w = pixel_to_world_coordinates(rectangle_obstacles_p, A_wp, b_wp).reshape(-1,4) # (x1,y1,x2,y2)
        free_range_w = min_max_aabb(free_range_w)
        rectangle_obstacles_w = min_max_aabb(rectangle_obstacles_w) # (x1, y1, x2, y2)
        rectangle_obstacles_w[:,2] -= rectangle_obstacles_w[:,0]
        rectangle_obstacles_w[:,3] -= rectangle_obstacles_w[:,1]
        circle_obstacles_p = np.array(map_config['circle_obstacles_pixel']) # (x, y, r)
        circle_obstacles_w =  pixel_to_world_coordinates(circle_obstacles_p[:,:2], A_wp, b_wp) # (x, y) # (n_circles, 2)
        circle_obstacles_w_radius = circle_obstacles_p[:,2:]*map_config['resolution'] # (n_circles, 1)
        circle_obstacles_w = np.concatenate([circle_obstacles_w, circle_obstacles_w_radius], axis=1)
        env_dict = {
            'x_range': (free_range_w[0], free_range_w[2]),
            'y_range': (free_range_w[1], free_range_w[3]),
            'circle_obstacles': circle_obstacles_w,
            'rectangle_obstacles': rectangle_obstacles_w,
        }
    return env_dict
    
def get_pose_msg(x, y, theta, frame_id='map', is_goal=False):
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    pose_msg.header.frame_id = frame_id
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    if is_goal:
        pose_msg.pose.position.z = 1. # * fake z for indicator of goal waypoint
    else:
        pose_msg.pose.position.z = 0.
    quaternion = tf_transformations.quaternion_from_euler(0, 0, theta) # theta radian
    pose_msg.pose.orientation.x = quaternion[0]
    pose_msg.pose.orientation.y = quaternion[1]
    pose_msg.pose.orientation.z = quaternion[2]
    pose_msg.pose.orientation.w = quaternion[3]
    return pose_msg


class GlobalPlanner(Node):
    def __init__(
        self,
        config,
    ):
        # * first ignore the lidar obstacles, only use static map
        super().__init__('png_navigation_global_planner')
        self.declare_parameter('use_neural_wrapper', False)
        self.declare_parameter('map', 'map_gazebo')

        self.env = Env(get_env_dict(self.get_parameter('map').get_parameter_value().string_value, get_package_share_directory('png_navigation'))) 
        self.use_neural_wrapper = self.get_parameter('use_neural_wrapper').get_parameter_value().bool_value
        self.config = config
        self.ros_config = config.ros_config
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.robot_pose = self.get_robot_pose()
        self.path = None
        self.x_goal = None
        self.goal_yaw = None
        self.goal_yaw_reached = False
        self.global_plan_pub = self.create_publisher(Path, '/global_plan', 10)
        self.global_plan_visual_pub = self.create_publisher(MarkerArray, 'png_navigation/global_plan', 10)
        self.waypoint_pub = self.create_publisher(PoseStamped, '/waypoint', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)
        self.se_client = self.create_client(SetEnv, 'png_navigation/set_env_2d')
        self.sen_client = self.create_client(SetEnv, 'png_navigation/neural_wrapper_set_env_2d')
        self.gp_client = self.create_client(GetGlobalPlan, 'png_navigation/get_global_plan')
        self.create_subscription(PoseStamped, self.ros_config.nav_goal_topic, self.nav_goal_callback, 10)
        self.create_subscription(Bool, '/waypoint_reached', self.waypoint_reached_callback, 10)
        self.path_waypoint_idx = 0
        self.set_environment()
        if self.use_neural_wrapper:
            self.set_environment_for_neural_wrapper()

    def set_environment(self):
        self.get_logger().info("Setting environment...")
        while not self.se_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('png_navigation/set_env_2d service not available, waiting again...')
        request = SetEnv.Request()
        request.request_env.num_dimensions = 2
        request.request_env.x_range = tuple(np.array(self.env.x_range, dtype=np.float64))
        request.request_env.y_range = tuple(np.array(self.env.y_range, dtype=np.float64))
        if self.env.obs_circle is None:
            request.request_env.circle_obstacles = []
        else:
            request.request_env.circle_obstacles = tuple(np.array(self.env.obs_circle, dtype=np.float64).flatten())
        if self.env.obs_rectangle is None:
            request.request_env.rectangle_obstacles = []
        else:
            request.request_env.rectangle_obstacles = tuple(np.array(self.env.obs_rectangle, dtype=np.float64).flatten())
        future = self.se_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().is_set:
            self.get_logger().info('Environment set successfully.')
        else:
            self.get_logger().error('Service call failed')
    
    def set_environment_for_neural_wrapper(self):
        self.get_logger().info("Intializing env in neural wrapper")
        while not self.sen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('png_navigation/neural_wrapper_set_env_2d service not available, waiting again...')
        request = SetEnv.Request()
        request.request_env.num_dimensions = 2
        request.request_env.x_range = tuple(np.array(self.env.x_range, dtype=np.float64))
        request.request_env.y_range = tuple(np.array(self.env.y_range, dtype=np.float64))
        if self.env.obs_circle is None:
            request.request_env.circle_obstacles = []
        else:
            request.request_env.circle_obstacles = tuple(np.array(self.env.obs_circle, dtype=np.float64).flatten())
        if self.env.obs_rectangle is None:
            request.request_env.rectangle_obstacles = []
        else:
            request.request_env.rectangle_obstacles = tuple(np.array(self.env.obs_rectangle, dtype=np.float64).flatten())
        future = self.sen_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Neural Wrapper Environment set successfully.')
        else:
            self.get_logger().error('Service call failed')

    def nav_goal_callback(self, msg):
        goal_position, goal_orientation = msg.pose.position, msg.pose.orientation
        angles = tf_transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.x_goal = [goal_position.x, goal_position.y]
        self.goal_yaw = angles[-1]
        self.plan()

    def waypoint_reached_callback(self, msg):
        if self.path_waypoint_idx == len(self.path)-1:
            goal_reached_msg = Bool()
            # Set the value of the Bool message
            goal_reached_msg.data = True  # Set to True or False based on your requirement
            self.goal_reached_pub.publish(goal_reached_msg)
            self.get_logger().info("Goal is reached.")
            return
        self.path_waypoint_idx += 1
        self.publish_waypoint()

    def plan_callback(self, future):
        if future.result() is not None and future.result().is_solved:
            self.path = np.array(future.result().path).reshape(-1, 2)  # np (n, 2)
            self.path_waypoint_idx = 1
            global_plan_msg = self.generate_path_msg(self.path)
            self.global_plan_pub.publish(global_plan_msg)
            self.publish_global_path_visual(self.path)
            self.get_logger().info("Published global plan")
            self.publish_waypoint()
        else:
            self.get_logger().info("Failure to find a global path is not implemented yet.")

    def get_robot_pose_callback(self, tf_future):
        try:
            trans = self.tf_buffer.lookup_transform('map', self.ros_config.robot_frame, rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=5.0))
            robot_x, robot_y = trans.transform.translation.x, trans.transform.translation.y
            euler = tf_transformations.euler_from_quaternion(
                [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z,
                    trans.transform.rotation.w])
            robot_theta = euler[2]
            self.robot_pose = [robot_x, robot_y, robot_theta]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.robot_pose = None
            self.get_logger().error(f"Error getting the transformation for robot pose.The reason is: {e}")
        self.get_logger().info(f'robot_pose: {self.robot_pose}')

    def plan(self):
        self.erase_path_visual()
        # (x, y)
        self.robot_pose = None
        while self.robot_pose is None:
            self.get_robot_pose()
        self.get_logger().info(f'robot_pose: {self.robot_pose}')
        x_start = self.robot_pose[:2]
        problem = {}
        problem['x_start'] = x_start
        problem['x_goal'] = self.x_goal
        problem['search_radius'] = np.float64(10) # * may be computed based on map, unit: m.
        problem['env'] = self.env

        while not self.gp_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('png_navigation/get_global_plan service not available, waiting again...')
        plan_request = GetGlobalPlan.Request()
        plan_request.plan_request.num_dimensions = 2
        plan_request.plan_request.start = x_start
        plan_request.plan_request.goal = self.x_goal
        plan_request.plan_request.search_radius = np.float64(10)  # * may be computed based on map, unit: m.
        plan_request.plan_request.clearance = self.config.robot_config.clearance_radius
        plan_request.plan_request.max_time = np.float64(self.config.path_planner_args.max_time)  # 5 second
        plan_request.plan_request.max_iterations = 50000
        self.gp_client.call_async(plan_request).add_done_callback(self.plan_callback)
    
    def publish_waypoint(self, frame_id="map"):
        if self.path is None:
            return
        if self.path_waypoint_idx == len(self.path)-1:
            x, y = self.path[self.path_waypoint_idx]
            theta = self.goal_yaw
            pose_msg = get_pose_msg(x, y, theta, frame_id=frame_id, is_goal=True)
            self.waypoint_pub.publish(pose_msg)
        else:
            x, y = self.path[self.path_waypoint_idx]
            theta = 0
            pose_msg = get_pose_msg(x, y, theta, frame_id=frame_id)
            self.waypoint_pub.publish(pose_msg)
    
    def get_robot_pose(self):
        self.tf_buffer.wait_for_transform_async('map', self.ros_config.robot_frame, rclpy.time.Time(seconds=0)).add_done_callback(self.get_robot_pose_callback)
    
    def publish_global_path_visual(self, path, frame_id="map"):
        path_visual_msg = MarkerArray()
        marker_id = 0
        for waypoint_start, waypoint_end  in zip(path[:-1], path[1:]):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.id = marker_id  # Each marker must have a unique ID
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.03#0.02
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            point1 = Point()
            point1.x = waypoint_start[0]
            point1.y = waypoint_start[1]
            point1.z = 0.05 # higher than tree
            point2 = Point()
            point2.x = waypoint_end[0]
            point2.y = waypoint_end[1]
            point2.z = 0.05 # higher than tree
            marker.points.append(point1)
            marker.points.append(point2)
            path_visual_msg.markers.append(marker)
            marker_id += 1
        self.global_plan_visual_pub.publish(path_visual_msg)
        return path

    def erase_path_visual(self):
        if self.path is not None:
            path_visual_msg = MarkerArray()
            for marker_id in range(len(self.path)-1):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.id = marker_id  # Each marker must have a unique ID
                marker.action = Marker.DELETE
                path_visual_msg.markers.append(marker)
                marker_id += 1
            self.global_plan_visual_pub.publish(path_visual_msg)

    @staticmethod
    def generate_path_msg(path, frame_id="map"):
        # Create a new nav_msgs/Path message
        path_msg = Path()
        # Set the header of the Path message
        path_msg.header.stamp = rclpy.time.Time().to_msg()  
        path_msg.header.frame_id = frame_id
        for waypoint in path:
            x, y = waypoint
            theta = 0
            pose_msg = get_pose_msg(x, y, theta, frame_id=frame_id)
            path_msg.poses.append(pose_msg)
        return path_msg

def main(args=None):
    rclpy.init(args=None)
    config = Config()
    gp = GlobalPlanner(
        config,
    )
    rclpy.logging.get_logger('global_planner_node:main').info("Global Planner is initialized.")
    try:
        rclpy.spin(gp)
    except KeyboardInterrupt:
        gp.get_logger().info('Node stopped cleanly')
    except Exception as e:
        gp.get_logger().error(f'Exception in node: {e}')
    finally:
        gp.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
