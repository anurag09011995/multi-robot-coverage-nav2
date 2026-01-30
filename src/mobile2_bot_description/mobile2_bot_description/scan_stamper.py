#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock
import time

class ScanStamper(Node):
    def __init__(self, robot_name):
        super().__init__(f'{robot_name}_scan_stamper')
        self.robot_name = robot_name
        self.last_clock_msg = None
        self.sim_time_ready= False

        #Qos for laser topics
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        #Qos for clock (explicitly RELIABLE)
        clock_qos = QoSProfile(
            reliability= ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.VOLATILE,
            depth=1
        )
        # self.create_timer(2.0, lambda: self._init_clock(clock_qos))
        self.clock_qos = clock_qos
        self.clock_init_timer = self.create_timer(2.0, self._init_clock)

        # self.create_subscription(Clock, '/clock', self.clock_callback, qos)
        # input from gazebo
        self.input_topic = f'/{robot_name}/scan_fixed'
        # output for SLAM
        self.output_topic = f'/{robot_name}/scan'

        self.create_subscription(
            LaserScan, self.input_topic, self.stamp_and_publish, scan_qos)
        # self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        
        self.pub = self.create_publisher(LaserScan, self.output_topic, scan_qos)
        self.get_logger().info(f"[{self.robot_name}] Scan stamper Initialized (delayed clock subscribe)")

    def _init_clock(self):

        if not hasattr(self, 'clock_sub'):
            self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, self.clock_qos)
            self.get_logger().info(f"[{self.robot_name}] Clock subscription created with RELIABLE QoS")
            self.clock_init_timer.cancel()

    def clock_callback(self, msg: Clock):
       self.last_clock_msg = msg

       self.sim_time_ready = True

    
    def stamp_and_publish(self, msg: LaserScan):

        # msg.header.stamp = Clock().now().to_msg()

        if not self.sim_time_ready or self.last_clock_msg is None:
            self.get_logger().warn(f"[{self.robot_name}] Waiting for /clock messages before stamping scans.")
            return
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.robot_name}lidar'

        # if not self.sim_time_ready:
        #     self.get_logger().warn("Waiting for /clock messages before stamping scans.")
        #     # skip until sim time is valid
        #     return
        # # ensure header is present
        # if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec ==0:
        #     msg.header.stamp = self.get_clock().now().to_msg()
        # # self.get_logger().info(f"{self.robot_name}: got a scan (frame={msg.header.frame_id or 'none'})")
        # if not msg.header.frame_id:
        #     msg.header.frame_id = f'{self.robot_name}lidar'
        self.pub.publish(msg)
        # self.get_logger().debug(f'{self.robot_name}: published stamped scan with time {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')

        #Log timming difference
        now_wall = time.time()
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        lag = now_wall - msg_time
        self.get_logger().info(f"[{self.robot_name}] Published scan lag = {lag:.3f}s")

def main(args=None):
    rclpy.init(args=args)
    node1 = ScanStamper('robot1')
    node2 = ScanStamper('robot2')

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node1.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
