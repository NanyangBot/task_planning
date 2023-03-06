import sys
import cv2
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from wall_painting_trajectory_planner.trajectory_planner import TrajectoryPlanner
from task_msgs.srv import DistanceMap, Trigger
from geometry_msgs.msg import PoseStamped

###############################################################################

class TrajectoryPlannerNode(Node):
    def __init__(self):
        super().__init__('trajectory_planner_node')

        self.busy = False
        self.planner = TrajectoryPlanner()

        self.path_pub = self.create_publisher(
            Path, 'wall_painting_trajectory_planner/path', 1)

        self.canvas_pub = self.create_publisher(
            MarkerArray, 'wall_painting_trajectory_planner/canvas', 1)

        self.trigger_srv = self.create_service(
            Trigger, 'wall_painting_trajectory_planner/trigger', self.task_cb)

        self.distance_map_cli = self.create_client(DistanceMap, 'wall_painting_trajectory_planner/distance_map')
        while not self.distance_map_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DistanceMap.Request()

        self.get_logger().info('Ready to accept Task Request')


    def send_request(self):
        self.future = self.canvas_cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def task_cb(self, request, response):
        if self.busy:
            response.success = False
            response.message = 'Busy with previous task request'
            return response

        self.busy = True
        input_task_image = cv2.imread(request.image_path,0)

        if input_task_image is None:
            response.success = False
            response.message = "Can't open/read input image file: check file"
            self.busy = False
            return response

        self.get_logger().info('Task received')

        distance_map = self.send_request()
        self.get_logger().info('Map received')

        self.planner.set_planning_scene(input_task_image, distance_map)

        path_msg = self.planner.get_path()
        self.path_pub.publish(path_msg)

        canvas_msg = self.planner.get_canvas()
        self.canvas_pub.publish(canvas_msg)

        response.success = True
        response.message = ''
        self.busy = False
        return response

###############################################################################

def main(argv=sys.argv):
    rclpy.init(args=argv)

    trajectory_planner_node = TrajectoryPlannerNode()
    rclpy.spin(trajectory_planner_node)

    trajectory_planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
