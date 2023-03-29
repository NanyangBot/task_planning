import sys
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import String
from task_msgs.msg import DistanceMapMsg
from visualization_msgs.msg import MarkerArray
from wall_painting_trajectory_planner.trajectory_planner import TrajectoryPlanner

###############################################################################

class TrajectoryPlannerNode(Node):
    def __init__(self):
        super().__init__('trajectory_planner_node')

        self.task = None
        self.dmap = None

        self.busy = False
        self.planner = TrajectoryPlanner(self.get_logger())

        self.wall_pub = self.create_publisher(
            MarkerArray, 'wall', 1)

        self.path_pub = self.create_publisher(
            Path, 'path', 1)

        self.dmap_sub = self.create_subscription(
            DistanceMapMsg, 'dmap', self.dmap_cb, 10)

        self.img_sub = self.create_subscription(
            String, 'image', self.image_cb, 10)

        self.get_logger().info('Ready to accept Task Request')

    def image_cb(self, msg):
        image = cv2.imread(msg.data,0)
        if image is None:
            message = "Can't open/read input image file: check file"
            self.get_logger().warn(message)
            return

        self.task = np.copy(image)
        self.get_logger().info('Task received')

    def dmap_cb(self, msg):
        if self.busy:
            message = 'Busy with previous task request'
            self.get_logger().warn(message)
            return

        self.busy = True
        self.dmap = msg
        self.get_logger().info('Distance Map received')

        self.planner.set_planning_scene(self.dmap)
        wall = self.planner.get_wall()
        self.wall_pub.publish(wall)

        self.planner.plan_task(self.task)
        path = self.planner.get_path()
        self.path_pub.publish(path)

        self.busy = False

###############################################################################

def main(argv=sys.argv):
    rclpy.init(args=argv)

    trajectory_planner_node = TrajectoryPlannerNode()
    rclpy.spin(trajectory_planner_node)

    trajectory_planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
