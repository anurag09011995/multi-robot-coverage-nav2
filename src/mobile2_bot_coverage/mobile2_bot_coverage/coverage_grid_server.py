import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf_transformations as tf
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class CoverageGridServer(Node):

    def __init__(self):
        super().__init__('coverage_grid_server')

        map_qos = QoSProfile(
            depth = 1,
            durability = DurabilityPolicy.TRANSIENT_LOCAL,
            reliability = ReliabilityPolicy.RELIABLE
        )

    
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        
        self.create_subscription(PoseWithCovarianceStamped, "/robot1/amcl_pose", self.robot1_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/robot2/amcl_pose", self.robot2_cb, 10)

        self.pub = self.create_publisher(OccupancyGrid, '/coverage_grid' , map_qos)

        self.coverage_data = None
        self.width = 0
        self.height = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0

        self.r1_pose = None
        self.r2_pose = None

        self.create_timer(0.5, self.timer_callback)

        self.get_logger().info("Simple Coverage Node Started. Waiting for /map...")

    def map_callback(self, msg):

        if self.coverage_data is None:
            self.width = msg.info.width
            self.height = msg.info.height
            self.resolution = msg.info.resolution
            self.origin_x = msg.info.origin.position.x
            self.origin_y = msg.info.origin.position.y

            self.coverage_data = [0] * (self.width * self.height)

            self.get_logger().info(f"Map Recieved! Size: {self.width}x{self.height}")


    def robot1_cb(self, msg):

        self.r1_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)


    def robot2_cb(self, msg):

        self.r2_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def timer_callback(self):
        if self.coverage_data is None:
            return
        
        # Paint r1
        if self.r1_pose:
            self.paint_spot(self.r1_pose[0], self.r1_pose[1])

        # Paint r2
        if self.r2_pose:
            self.paint_spot(self.r2_pose[0], self.r2_pose[1])
        
        self.publish_grid()


    def paint_spot(self, x, y):

        if self.coverage_data is None:
            return


        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)

        brush_radius = 5
        for i in range(-brush_radius, brush_radius + 1):       # -1, 0, 1
            for j in range(-brush_radius, brush_radius + 1):   # -1, 0, 1

                # Calculate index
                check_x = grid_x + i
                check_y = grid_y + j

                # check boundaries so we dont crash
                if 0 <= check_x < self.width and 0 <= check_y < self.height:
                    index = check_y * self.width + check_x
                    if self.coverage_data[index] != 100:
                        self.coverage_data[index] = 100


    def publish_grid(self):

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y

        msg.data = self.coverage_data
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CoverageGridServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

