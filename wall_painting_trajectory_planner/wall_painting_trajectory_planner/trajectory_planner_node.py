import sys
import cv2
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
from task_msgs.srv import DistanceMapSrv, TaskPlanning
from rclpy.callback_groups import ReentrantCallbackGroup
from wall_painting_trajectory_planner.trajectory_planner import TrajectoryPlanner

###############################################################################

class TrajectoryPlannerNode(Node):
    def __init__(self):
        super().__init__('trajectory_planner_node')

        self.busy = False
        self.planner = TrajectoryPlanner()
        group_1 = ReentrantCallbackGroup()
        group_2 = ReentrantCallbackGroup()
        self.trigger_srv = self.create_service(
            TaskPlanning, 'wall_painting_trajectory_planner/trigger', self.task_cb, callback_group=group_1)

        self.distance_map_cli = self.create_client(DistanceMapSrv, 'vision_preprocess_service_task', callback_group=group_2)
        while not self.distance_map_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service not available, waiting again...')
        self.req = DistanceMapSrv.Request()

        self.wall_pub = self.create_publisher(
            MarkerArray, 'wall', 1)

        self.path_pub = self.create_publisher(
            Path, 'path', 1)

        self.get_logger().info('Ready to accept Task Request')


    def send_request(self):
        self.future = self.distance_map_cli.call_async(self.req)
        while not self.future.done():
           time.sleep(0.01)
        return self.future.result()

    def task_cb(self, request, response):
        if self.busy:
            response.success = False
            response.message = 'Busy with previous task request'
            self.get_logger().warn(response.message)
            return response

        self.busy = True
        input_task_image = cv2.imread(request.task_file_location,0)

        if input_task_image is None:
            response.success = False
            response.message = "Can't open/read input image file: check file"
            self.get_logger().warn(response.message)
            self.busy = False
            return response

        self.get_logger().info('Task received')

        vision_response = self.send_request()
        #print("here-------")
        #print(vision_response.success)

        if not vision_response.success:
            response.success = False
            response.message = "Failed to get map from vision service"
            self.get_logger().warn(response.message)
            self.busy = False
            return response

        self.get_logger().info('Map received')
        distance_map = vision_response.dmap

        self.planner.set_planning_scene(distance_map)
        wall = self.planner.get_wall()
        self.wall_pub.publish(wall)
        
        self.planner.plan_task(input_task_image)
        path = self.planner.get_path()
        self.path_pub.publish(path)

        response.success = True
        response.path = path
        response.wall = wall
        
        #self.path_pub.publish(path)
        #self.wall_pub.publish(wall)

        self.busy = False
        return response

###############################################################################

def main(argv=sys.argv):
    rclpy.init(args=argv)

    exec = MultiThreadedExecutor()

    trajectory_planner_node = TrajectoryPlannerNode()
    # rclpy.spin(trajectory_planner_node)

    exec.add_node(trajectory_planner_node)
    exec.spin()
    exec.shutdown()


    trajectory_planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
