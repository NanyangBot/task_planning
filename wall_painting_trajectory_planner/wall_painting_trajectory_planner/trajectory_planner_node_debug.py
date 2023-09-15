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

import yaml

def save_path_message_as_yaml(msg):
    # msg_dict = {field: (getattr(msg, field) if isinstance(getattr(msg, field), str) else float(getattr(msg, field))) for field in msg._fields_and_field_types if hasattr(msg, field)}

    pose_dict_list = []
    for p in msg.poses:
        x = float(p.pose.position.x)
        y = float(p.pose.position.y)
        z = float(p.pose.position.z)
        qx = float(p.pose.orientation.x)
        qy = float(p.pose.orientation.y)
        qz = float(p.pose.orientation.z)
        qw = float(p.pose.orientation.w)
        frame_id = p.header.frame_id
        pose_dict = {'header':{'frame_id':frame_id},
                     'pose':{'position':{'x':x, 'y':y, 'z':z},
                             'orientation':{'x':qx, 'y':qy, 'z':qz, 'w':qw}}}
        pose_dict_list.append(pose_dict)
    frame_id = msg.header.frame_id
    dict_file = {'header':{'frame_id':frame_id}, 'poses':pose_dict_list}

    # Save path as a YAML file in the location from where you're running this node
    with open('output_file.yaml', 'w') as file:
        out = yaml.dump(dict_file, file)

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

        save_path_message_as_yaml(path)
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
