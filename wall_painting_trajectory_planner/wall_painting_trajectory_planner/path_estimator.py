import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from wall_painting_trajectory_planner.DistanceMap import DistanceMap
from task_msgs.msg import DistanceMapSlice

###############################################################################

class PathEstimator(Node):
    def __init__(self):
        super().__init__('path_estimator')

        self.busy = False

        self.task_path_pub = self.create_publisher(
            Path, 'task_path', 1)

        self.wall_pub = self.create_publisher(
            MarkerArray, 'wall', 1)

        self.task_sub = self.create_subscription(
            DistanceMapSlice, 'task', self.task_cb, 10)

        self.get_logger().info('Ready to accept Task Request')

    def task_cb(self, msg: DistanceMapSlice):
        self.get_logger().info('Task received')
        if msg.task and msg.data:
            if not self.busy:
                self.busy = True
                
                self.get_logger().info('Computing path...')
                wall_map = DistanceMap(msg)

                wall_msg = MarkerArray()

                for i in range(msg.height):
                    for j in range(msg.width):
                        m = Marker()
                        m.id = int(str(i)+str(j))
                        m.pose.position = wall_map.get_position_at_(i,j)
                        m.header.stamp = self.get_clock().now().to_msg()
                        m.header.frame_id='map'
                        m.scale.x = 0.2
                        m.scale.y = 0.2
                        m.scale.z = 0.2
                        m.color.r = 0.0
                        m.color.g = 1.0
                        m.color.b = 0.0
                        m.color.a = 1.0
                        m.type = 2
                        wall_msg.markers.append(m)
                self.wall_pub.publish(wall_msg)

                path_msg = Path()
                path_msg.header.stamp = self.get_clock().now().to_msg()
                path_msg.header.frame_id='map'
                path_msg.poses = wall_map.get_path()
                self.task_path_pub.publish(path_msg)

                self.busy = False
        else:
            self.get_logger().warning('Ignoring...')

###############################################################################

def main(argv=sys.argv):
    rclpy.init(args=argv)

    path_estimator = PathEstimator()
    rclpy.spin(path_estimator)

    path_estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
