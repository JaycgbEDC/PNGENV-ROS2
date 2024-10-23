#!/usr/bin/python3.8
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LocalPlannerClockNode(Node):
    def __init__(self):
        super().__init__('png_navigation_local_planner_clock')
        self.publisher_ = self.create_publisher(String, 'png_navigation/local_planner_clock', 10)
        self.desired_frequency = 20
        self.timer = self.create_timer(1.0 / self.desired_frequency, self.timer_callback)

    def timer_callback(self):
        message = String()
        message.data = "dummy msg for local planner"
        self.publisher_.publish(message)

def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerClockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    except Exception as e:
        node.get_logger().error(f'Exception in node: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()