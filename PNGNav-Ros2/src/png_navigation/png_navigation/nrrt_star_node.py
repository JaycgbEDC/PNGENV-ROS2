#!/home/sora/anaconda3/envs/pngenv/bin/python
import rclpy
import numpy as np
from rclpy.node import Node

from png_navigation.tools.configs.rrt_star_config import Config
from png_navigation.tools.path_planning_classes.rrt_env_2d import Env
from png_navigation.tools.path_planning_classes.nrrt_star_png_2d import get_path_planner

from png_interfaces.srv import SetEnv
from png_interfaces.srv import GetGlobalPlan


def get_fake_env():
    env_dict = {
        'x_range': [0,10],
        'y_range': [0,10],
        'circle_obstacles': [],
        'rectangle_obstacles': [],
    }
    return Env(env_dict)

def get_fake_problem():
    problem = {
        'x_start': [0,0],
        'x_goal': [1,1],
        'search_radius': 10,
        'env': get_fake_env(),
    }
    return problem

class NRRTStarNode(Node):
    def __init__(
        self,
    ):
        super().__init__('png_navigation_nrrt_star_node')
        self.config = Config()
        self.planner = get_path_planner(
            self.config.path_planner_args,
            get_fake_problem(),
        )
        self.set_env_service = self.create_service(SetEnv, 'png_navigation/set_env_2d', self.set_env)
        self.get_global_plan_service = self.create_service(GetGlobalPlan, 'png_navigation/get_global_plan', self.get_global_plan)
    
    def get_global_plan(self, request, response):
        self.planner.reset_robot(
            x_start=request.plan_request.start,
            x_goal=request.plan_request.goal,
            env=None,
            search_radius=request.plan_request.search_radius,
            max_time=request.plan_request.max_time,
        )
        # * clearance and max_iterations from plan_request are redundant and not used here.
        path = self.planner.planning_robot()
        if len(path) == 0:
            response.is_solved = False
            response.path = []
        else:
            response.is_solved = True
            response.path = path.flatten().tolist()
        return response

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
        self.planner.reset_env_robot(Env(env_dict))
        self.get_logger().info("Environment is set.")
        response.is_set = True
        return response
    

def main(args=None):
    rclpy.init(args=args)
    nrrtsn = NRRTStarNode()
    try:
        rclpy.spin(nrrtsn)
    except KeyboardInterrupt:
        nrrtsn.get_logger().info('Node stopped cleanly')
    except Exception as e:
        nrrtsn.get_logger().error(f'Exception in node: {e}')
    finally:
        nrrtsn.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()