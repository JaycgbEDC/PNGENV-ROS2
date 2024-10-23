#!/usr/bin/python3.8
import math
import numpy as np

import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
import tf2_ros

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, Point, PoseStamped


def normalize_angle(angle):
    normalized_angle = math.fmod(angle + math.pi, 2 * math.pi)
    if normalized_angle < 0:
        normalized_angle += 2*math.pi
    return normalized_angle - math.pi

class LocalPlanner(Node):
    def __init__(
        self,
        robot_frame='base_footprint',
        linear_speed_levels=(0.05, 0.1, 0.2),
        angular_speed_levels=(0.2, 0.4),
        distance_threshold=(0.05, 0.3),
        angle_threshold=(0.05, 0.1),
        linear_speed_increment=0.01,
        angular_speed_increment=0.1,
    ):
        super().__init__('png_navigation_local_planner')
        self.cmd_vel = self.create_publisher(Twist, 'cmd_vel', 5) # gazebo
        # * self.cmd_vel = self.create_publisher(Twist, 'cmd_vel_mux/input/teleop', 5) # real world
        self.waypoint_reached_pub = self.create_publisher(Bool, '/waypoint_reached', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.odom_frame = 'map'
        self.base_frame = robot_frame

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0
        self.is_global_goal = False

        self.linear_speed_levels = linear_speed_levels
        self.angular_speed_levels = angular_speed_levels
       
        self.linear_speed_increment = linear_speed_increment # 0.01#0.005
        self.angular_speed_increment = angular_speed_increment # 0.1
        self.linear_speed = 0
        self.angular_speed = 0

        self.target_linear_speed = 0
        self.target_angular_speed = 0

        self.distance_threshold = distance_threshold
        self.angle_threshold = angle_threshold
        self.drive_robot = False

        self.create_subscription(PoseStamped, '/waypoint', self.waypoint_callback, 10)
        self.create_subscription(String, 'png_navigation/local_planner_clock', self.clock_callback, 10)
        
        self.get_logger().info("Local Planner is initialized.")

    def waypoint_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        if msg.pose.position.z != 0:
            self.is_global_goal = True
            angles = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            self.goal_yaw = angles[-1]
        self.drive_robot = True
    
    def clock_callback(self, msg):
        if not self.drive_robot:
            return
        position, rotation = self.get_pose()
        distance = np.sqrt((self.goal_x - position.x)**2 + (self.goal_y - position.y)**2)
        msg_publish = Bool()
     
        if distance < self.distance_threshold[0]:
            if not self.is_global_goal:
                self.get_logger().info("Waypoint reached.")
                msg_publish.data = True
                self.waypoint_reached_pub.publish(msg_publish)
                return
            else:
                self.target_linear_speed = 0
                remaining_rotation = normalize_angle(self.goal_yaw - rotation)
                if abs(remaining_rotation) > self.angle_threshold[1]:
                    if remaining_rotation > 0:
                        self.target_angular_speed = self.angular_speed_levels[1] # 0.4
                    else:
                        self.target_angular_speed = -self.angular_speed_levels[1] # -0.4
                    self.send_velocity_command(self.target_linear_speed, self.target_angular_speed)
                    return
                elif abs(remaining_rotation) > self.angle_threshold[0]:
                    if remaining_rotation > 0:
                        self.target_angular_speed = self.angular_speed_levels[0] # 0.2
                    else:
                        self.target_angular_speed = -self.angular_speed_levels[0] # -0.2
                    self.send_velocity_command(self.target_linear_speed, self.target_angular_speed)
                    return
                else:
                    self.target_angular_speed = 0
                    self.send_velocity_command(self.target_linear_speed, self.target_angular_speed)
                    if self.linear_speed==0 and self.angular_speed==0:
                        self.get_logger().info("Global goal reached announced by local planner.") # global goal reached
                        msg_publish.data = True
                        self.waypoint_reached_pub.publish(msg_publish)
                        self.drive_robot = False
                        self.goal_yaw = None
                        self.is_global_goal = False
                        return
        
        path_angle = np.arctan2(self.goal_y - position.y, self.goal_x - position.x)
        remaining_rotation = normalize_angle(path_angle - rotation)
        if abs(remaining_rotation) > self.angle_threshold[1]:
            self.target_linear_speed = 0.
            if remaining_rotation > 0:
                self.target_angular_speed = self.angular_speed_levels[1] # 0.4
            else:
                self.target_angular_speed = -self.angular_speed_levels[1] # -0.4
            self.send_velocity_command(self.target_linear_speed, self.target_angular_speed)
            return
        elif abs(remaining_rotation) > self.angle_threshold[0]:
            if remaining_rotation > 0:
                self.target_angular_speed = self.angular_speed_levels[0] # 0.2
            else:
                self.target_angular_speed = -self.angular_speed_levels[0] # -0.2

            if distance < self.distance_threshold[1] and self.is_global_goal:
                self.target_linear_speed = self.linear_speed_levels[0] # 0.05
            else:
                self.target_linear_speed = self.linear_speed_levels[2] # 0.2
            self.send_velocity_command(self.target_linear_speed, self.target_angular_speed)
            return
        else:
            self.target_angular_speed = 0.
            if distance < self.distance_threshold[1] and self.is_global_goal:
                self.target_linear_speed = self.linear_speed_levels[1] # 0.1
            else:
                self.target_linear_speed = self.linear_speed_levels[2] # 0.2
            self.send_velocity_command(self.target_linear_speed, self.target_angular_speed)
            return
            
    def send_velocity_command(self, target_linear_speed, target_angular_speed):
        if abs(target_linear_speed-self.linear_speed) < self.linear_speed_increment:
            self.linear_speed = target_linear_speed
        else:
            self.linear_speed += self.linear_speed_increment*(target_linear_speed-self.linear_speed)/abs(self.target_linear_speed-self.linear_speed)
        if abs(target_angular_speed-self.angular_speed) < self.angular_speed_increment:
            self.angular_speed = target_angular_speed
        else:
            self.angular_speed += self.angular_speed_increment*(target_angular_speed-self.angular_speed)/abs(self.target_angular_speed-self.angular_speed)
        move_cmd = Twist()
        move_cmd.linear.x = np.float64(self.linear_speed)
        move_cmd.angular.z = np.float64(self.angular_speed)
        self.cmd_vel.publish(move_cmd)

    def get_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time())
            trans = transform.transform.translation
            rot = transform.transform.rotation
            rotation = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        except (tf2_ros.Exception, tf2_ros.ConnectivityException, tf2_ros.LookupException):
            self.get_logger().info("TF Exception")
            return
        return Point(x=trans.x, y=trans.y, z=trans.z), rotation[2]

    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rclpy.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    lp = LocalPlanner()
    try:
        rclpy.spin(lp)
    except KeyboardInterrupt:
        lp.get_logger().info('Node stopped cleanly')
    except Exception as e:
        lp.get_logger().error(f'Exception in node: {e}')
    finally:
        lp.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


