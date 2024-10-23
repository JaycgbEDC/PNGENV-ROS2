#!/home/sora/anaconda3/envs/pngenv_ros2/bin/python
import math
import time

import rclpy
from rclpy.node import Node
import numpy as np
import tf2_geometry_msgs
from tf_transformations import euler_from_quaternion
import tf2_ros
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from png_navigation.tools.path_planning_classes.rrt_env_2d import Env
from png_navigation.tools.path_planning_classes.rrt_utils_2d import Utils

from std_msgs.msg import Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped, PoseArray
from visualization_msgs.msg import Marker, MarkerArray

from png_interfaces.srv import GlobalReplan
from png_interfaces.srv import StopRobot, ResumeRobot


def normalize_angle(angle):
    normalized_angle = math.fmod(angle + math.pi, 2 * math.pi)
    if normalized_angle < 0:
        normalized_angle += 2*math.pi
    return normalized_angle - math.pi

def get_fake_env():
    env_dict = {
        'x_range': [-100,100],
        'y_range': [-100,100],
        'circle_obstacles': [],
        'rectangle_obstacles': [],
    }
    return Env(env_dict)

class HumanChecker(Node):
    def __init__(
        self,
        robot_frame='base_footprint',
        human_detection_radius=0.5,
        human_clearance=0.1,
    ):
        super().__init__('human_checker')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.odom_frame = 'map'
        self.base_frame = robot_frame

        self.human_detection_radius = human_detection_radius # * meter
        self.human_clearance = human_clearance
        self.humans = np.array([])
        self.interactive_humans = np.array([])

        self.global_path = np.array([])
        self.path_waypoint_idx = 0 # next waypoint index
        self.human_marker_pub = self.create_publisher(MarkerArray, "human_marker", 10)

        self.global_plan_sub = self.create_subscription(Path, '/global_plan', self.global_plan_callback, 10)
        self.waypoint_reached_sub = self.create_subscription(Bool, '/waypoint_reached', self.waypoint_reached_callback, 10)
        self.human_sub = self.create_subscription(PoseArray, 'dr_spaam_detections', self.human_callback, 1) # * static human for now
        self.stop_robot_client = self.create_client(StopRobot, 'png_navigation/stop_robot')
        self.global_replan_client = self.create_client(GlobalReplan, 'png_navigation/global_replan')
        self.resume_robot_client = self.create_client(ResumeRobot, 'png_navigation/resume_robot')

        self.get_logger().info("Human Checker is initialized.")

        self.need_global_replan = False
        self.human_obstacles = []


    def global_plan_callback(self, msg):
        self.global_path = np.array([
            (single_pose.pose.position.x, single_pose.pose.position.y)
            for single_pose in msg.poses])
        self.path_waypoint_idx = 1
    
    def waypoint_reached_callback(self, msg):
        if self.path_waypoint_idx == len(self.global_path)-1:
            # * goal is reached
            self.global_path = np.array([])
            self.path_waypoint_idx = 0 # next waypoint index
            return
        self.path_waypoint_idx += 1
                    
    def get_pose(self):
        try:
            now = rclpy.time.Time(seconds=0)
            trans = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, now)
            rotation = euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
            return Point(x=trans.transform.translation.x, y=trans.transform.translation.y, z=trans.transform.translation.z), rotation[2]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().info("TF Exception")
            return

    def stop_robot_callback1(self, future):
        if future.result().is_stopped:
            self.get_logger().info("Robot is stopped upon path collision with human.")
        else:
            self.get_logger().info("Robot is not stopped upon path collision with human.")
        while not self.global_replan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('png_navigation/global_replan service not available, waiting again...')
        req = GlobalReplan.Request()
        req.replan_human_obstacles = []  # empty
        self.global_replan_client.call_async(req).add_done_callback(self.global_replan_callback1)

    def stop_robot_callback2(self, future):
        if future.result().is_stopped:
            self.get_logger().info("Robot is stopped upon path collision with human.")
            while not self.global_replan_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('png_navigation/global_replan service not available, waiting again...')
            req = GlobalReplan.Request()
            req.replan_human_obstacles = list(self.human_obstacles.reshape(-1))
            self.global_replan_client.call_async(req).add_done_callback(self.global_replan_callback2)
        else:
            self.get_logger().info("Robot is not stopped upon path collision with human.")

    def global_replan_callback1(self, future):
        if future.result().is_replanned:
            self.get_logger().info("Replanned global path.")
            self.need_global_replan = False
        else:
            self.get_logger().info("Failure to replan a global path.")

    def global_replan_callback2(self, future):
        if future.result().is_replanned:
            self.get_logger().info("Replanned global path.")
            self.need_global_replan = False
            time.sleep(5)  # added by lx（不加会鬼畜，即规划好路径后还没走又会规划）
        else:
            self.get_logger().info("Failure to replan a global path.")

    def resume_robot_callback(self, future):
        if future.result().is_resumed:
            self.get_logger().info("Robot is resumed upon no path collision with human.")
        else:
            self.get_logger().info("Robot is not resumed upon no path collision with human.")

    def human_callback(self, msg):
        self.get_logger().info('test1')
        try:
            poses_with_headers = [PoseStamped(header=msg.header, pose=pose) for pose in msg.poses]
            # (trans, rot) = self.tf_listener.lookupTransform('map', 'laser', rospy.Time(0)) # ! for real robot
            trans = self.tf_buffer.lookup_transform('map', 'map', rclpy.time.Time())  # for gazebo simulation
            transformed_posi = [self.tf_buffer.transform(pose, "map").pose.position for pose in poses_with_headers]
            if len(self.humans)>0: # original
                human_marker_msg = MarkerArray()
                for human_id in range(len(self.humans)):
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.id = human_id  # Each marker must have a unique ID
                    marker.action = Marker.DELETE
                    human_marker_msg.markers.append(marker)
                self.human_marker_pub.publish(human_marker_msg)

            self.humans = np.array([[pose.x, pose.y] for pose in transformed_posi]) # (n, 2)
            human_marker_msg = MarkerArray()
            for human_id, human in enumerate(self.humans):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.ns = "human"
                marker.id = human_id
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                marker.pose.position.x = human[0]
                marker.pose.position.y = human[1]
                marker.pose.position.z = 0.0  # Z coordinate
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = (0.28+0.01)*2 # 1.0  # Radius of the circle
                marker.scale.y = (0.28+0.01)*2 # 1.0  # Radius of the circle
                marker.scale.z = 0.01
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                human_marker_msg.markers.append(marker)
            self.human_marker_pub.publish(human_marker_msg)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("Transform lookup failed.")
        
        if len(self.global_path) == 0:
            # not planned yet
            return

        robot_position, robot_rotation = self.get_pose()
        if robot_position is None:
            self.get_logger().warn("Robot pose not available.")
            return
        robot_pos = np.array([robot_position.x, robot_position.y]) # (2,)

        if len(self.humans)==0:
            if self.need_global_replan:
                while not self.stop_robot_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('png_navigation/stop_robot service not available, waiting again...')
                req = StopRobot.Request()
                self.stop_robot_client.call_async(req).add_done_callback(self.stop_robot_callback1)
        else:
            humans_to_robot_dist= np.linalg.norm(self.humans-robot_pos, axis=1) # (n,)
            if np.all(humans_to_robot_dist>self.human_detection_radius) \
                and not self.need_global_replan:              
                return
            self.interactive_humans = self.humans[np.where(humans_to_robot_dist<=self.human_detection_radius)] # (m, 2)
            self.human_obstacles = np.concatenate(
                [self.interactive_humans,
                np.ones((len(self.interactive_humans), 1))*self.human_clearance],
                axis=1,
            ) # (m, 3)
            env_dict = {
                'x_range': [-100,100], # * dummy
                'y_range': [-100,100], # * dummy
                'circle_obstacles': self.human_obstacles,
                'rectangle_obstacles': [],
            }
            # utils = Utils(Env(env_dict), 0) # * human clearance added to circle radius
            # utils = Utils(Env(env_dict), 0.18) # ! need adjustment on rrt_config human clearance equal robot radius 
            utils = Utils(Env(env_dict), 0.15) # ! need adjustment on rrt_config human clearance equal robot radius 
            if utils.is_inside_obs(robot_pos):
                self.need_global_replan = True
                self.get_logger().info("Robot is within clearance of human obstacles.")
                return

            current_path = np.concatenate([robot_pos[np.newaxis,:], self.global_path[self.path_waypoint_idx:]], axis=0)
            collision = False
            for i in range(len(current_path)-1):
                if np.linalg.norm(current_path[i]-robot_pos)>self.human_detection_radius and \
                    np.linalg.norm(current_path[i+1]-robot_pos)>self.human_detection_radius:
                    continue
                if utils.is_collision(current_path[i], current_path[i+1]):
                    collision = True
                    break
            if not collision:
                self.need_global_replan = False
                while not self.resume_robot_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('png_navigation/resume_robot service not available, waiting again...')
                req = ResumeRobot.Request()
                self.resume_robot_client.call_async(req).add_done_callback(self.resume_robot_callback)
            else:
                self.need_global_replan = True
                while not self.stop_robot_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('png_navigation/stop_robot service not available, waiting again...')
                req = StopRobot.Request()
                self.stop_robot_client.call_async(req).add_done_callback(self.stop_robot_callback2)                 
    

def main(args=None):
    rclpy.init(args=args)
    hc = HumanChecker()
    try:
        rclpy.spin(hc)
    except KeyboardInterrupt:
        hc.get_logger().info('Node stopped cleanly')
    except Exception as e:
        hc.get_logger().error(f'Exception in node: {e}')
    finally:
        hc.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()