#!/home/sora/anaconda3/envs/pngenv/bin/python
import struct
from os.path import join

import rclpy
from rclpy.node import Node
import numpy as np
from ament_index_python.packages import get_package_share_directory

from png_navigation.tools.configs.rrt_star_config import Config
from png_navigation.tools.path_planning_classes.rrt_env_2d import Env
from png_navigation.tools.wrapper.pointnet_pointnet2.pointnet2_wrapper_connect_bfs import PNGWrapper as NeuralWrapper
from png_navigation.tools.datasets.point_cloud_mask_utils_updated import generate_rectangle_point_cloud, ellipsoid_point_cloud_sampling

import std_msgs.msg
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import PointCloud2, PointField

from png_interfaces.msg import NIRRTWrapperMsg
from png_interfaces.srv import SetEnv


class NeuralWrapperNode(Node):
    def __init__(
        self,
        pc_n_points,
        pc_over_sample_scale,
        clearance,
        step_len,
        connect_max_trial_attempts,
    ):
        super().__init__('png_navigation_nirrt_star_c_neural_wrapper_node')
        package_name = 'png_navigation'
        root_folderpath = get_package_share_directory(package_name)  
        self.neural_wrapper = NeuralWrapper(
            root_dir=root_folderpath,
            device='cuda',
        )
        self.pc_n_points = pc_n_points
        self.pc_over_sample_scale = pc_over_sample_scale
        self.pc_neighbor_radius = step_len
        self.clearance = clearance
        self.connect_max_trial_attempts = connect_max_trial_attempts
        self.pub = self.create_publisher(Float64MultiArray, 'wrapper_output', 10)
        self.guidance_states_pub = self.create_publisher(PointCloud2, 'guidance_states', 10)
        self.no_guidance_states_pub = self.create_publisher(PointCloud2, 'no_guidance_states', 10)
        self.create_subscription(NIRRTWrapperMsg, 'wrapper_input', self.callback, 10) # * throw away outdated messages
        self.create_service(SetEnv, 'png_navigation/neural_wrapper_set_env_2d', self.set_env)

    def set_env(self, request, response):
        if len(request.request_env.circle_obstacles)>0:
            circle_obstacles = np.array(request.request_env.circle_obstacles).reshape(-1,3)
        else:
            circle_obstacles = []
        if len(request.request_env.rectangle_obstacles)>0:
            rectangle_obstacles = np.array(request.request_env.rectangle_obstacles).reshape(-1,4)
        else:
            rectangle_obstacles = []       
        env_dict = {
            'x_range': request.request_env.x_range,
            'y_range': request.request_env.y_range,
            'circle_obstacles': circle_obstacles,
            'rectangle_obstacles': rectangle_obstacles,
        }
        self.env = Env(env_dict)
        self.get_logger().info("Environment is set for Neural Wrapper.")
        response.is_set = True
        return response
    
    def callback(self, msg):
        self.get_logger().info("Received request for point net guide inference.")
        x_start = np.array(msg.x_start).astype(np.float64)
        x_goal = np.array(msg.x_goal).astype(np.float64)
        cmax = msg.c_max
        cmin = msg.c_min
        if cmax < np.inf:
            max_min_ratio = cmax/cmin
            pc = ellipsoid_point_cloud_sampling(
                x_start,
                x_goal,
                max_min_ratio,
                self.env,
                n_points=self.pc_n_points,
                n_raw_samples=self.pc_n_points*self.pc_over_sample_scale,
                clearance=self.clearance,
            )
        else:
            pc = generate_rectangle_point_cloud(
                self.env,
                self.pc_n_points,
                over_sample_scale=self.pc_over_sample_scale,
                use_open3d=True,
                clearance=self.clearance,
            )
        fake_env_dict = {}
        fake_env_dict['env_dims'] = [10,10]
        connection_success, _, path_pred = self.neural_wrapper.generate_connected_path_points(
            pc.astype(np.float32),
            x_start,
            x_goal,
            fake_env_dict,
            neighbor_radius=self.pc_neighbor_radius,
            max_trial_attempts=self.connect_max_trial_attempts,
        )
        self.get_logger().info(f"Connection success: {connection_success}")
        self.path_point_cloud_pred = pc[path_pred.nonzero()[0]] # (<pc_n_points, 2)
        msg = Float64MultiArray()
        msg.data = self.path_point_cloud_pred.flatten().tolist()
        self.pub.publish(msg)

        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1)
        ]
        points = []
        r = int(1*255)
        g = int(0.4980392156862745*255)
        b = int(0.054901960784313725*255)
        a = 255
        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        for i in range(len(self.path_point_cloud_pred)):
            x = self.path_point_cloud_pred[i,0]
            y = self.path_point_cloud_pred[i,1]
            z = 0
            points.append([x, y, z, rgb])
        cloud_msg = point_cloud2.create_cloud(header, fields, points)
        self.guidance_states_pub.publish(cloud_msg)

        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1)
        ]
        points = []
        r = 49
        g = 130
        b = 189
        a = 255
        other_point_cloud = pc[(1-path_pred).nonzero()[0]]
        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        for i in range(len(other_point_cloud)):
            x = other_point_cloud[i,0]
            y = other_point_cloud[i,1]
            z = 0
            points.append([x, y, z, rgb])
        cloud_msg = point_cloud2.create_cloud(header, fields, points)
        self.no_guidance_states_pub.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    config = Config()
    nwn = NeuralWrapperNode(
        config.png_config.pc_n_points,
        config.png_config.pc_over_sample_scale,
        config.png_config.clearance,
        config.png_config.step_len,
        config.png_config.connect_max_trial_attempts,
    )
    try:
        rclpy.spin(nwn)
    except KeyboardInterrupt:
        nwn.get_logger().info('Node stopped cleanly')
    except Exception as e:
        nwn.get_logger().error(f'Exception in node: {e}')
    finally:
        nwn.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()     