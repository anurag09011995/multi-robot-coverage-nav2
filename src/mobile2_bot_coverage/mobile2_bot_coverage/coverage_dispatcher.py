import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import threading
from collections import deque
import math
import random
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class MultiRobotDispatcher(Node):
    def __init__(self):
        super().__init__('multi_robot_dispatcher')

        self.cb_group = ReentrantCallbackGroup()

        map_qos = QoSProfile(
            depth = 10,
            durability = DurabilityPolicy.TRANSIENT_LOCAL,
            reliability = ReliabilityPolicy.RELIABLE
        )

        #subscribers
        self.create_subscription(OccupancyGrid, '/coverage_grid', self.coverage_cb, 10, callback_group=self.cb_group)
        self.create_subscription(OccupancyGrid, '/map', self.static_map_cb, map_qos, callback_group=self.cb_group)

        #New: Listen to Robot Positions(Needed to find "Nearest" goal)
        self.create_subscription(PoseWithCovarianceStamped, '/robot1/amcl_pose', self.r1_pose_cb, 10, callback_group=self.cb_group)
        self.create_subscription(PoseWithCovarianceStamped, '/robot2/amcl_pose', self.r2_pose_cb, 10, callback_group=self.cb_group)

        self.r1_init_pub = self.create_publisher(PoseWithCovarianceStamped, '/robot1/initialpose', 10)
        self.r2_init_pub = self.create_publisher(PoseWithCovarianceStamped, '/robot2/initialpose', 10)


        # Nav2 navigators

        self.nav1 = BasicNavigator(namespace='robot1')
        self.nav2 = BasicNavigator(namespace='robot2')

        # Data Storage
        self.coverage_data = None
        self.static_map_data = None
        self.map_info = None

        # Robot Positions (updated by amcl)
        self.r1_pose = None
        self.r2_pose = None

        # Concurrency & Safety
        self.lock = threading.RLock()

        self.reserved_indices = set()
        self.fail_counts = {}
        self.blacklisted = set()

        self.r1_current_idx = None
        self.r2_current_idx = None

        #Initialization
        self.nav_initialized = False
        t = threading.Thread(target=self._wait_for_nav2, daemon= True)
        t.start()



    #     # Main Loop

        self.timer = self.create_timer(1.0, self.control_loop, callback_group=self.cb_group)
        self.get_logger().info("Intelligent Dispatcher Started. Waiting for data...")
        
    def _wait_for_nav2(self):
        self.get_logger().info('Waiting 5 secs for system to warm up...')
        time.sleep(2.0)
        # self.nav1.waitUntilNav2Active()
        # self.nav2.waitUntilNav2Active()
        self.nav_initialized = True
        self.get_logger().info('Nav2 Ready! Intelligent Coverage Active.')

        self.get_logger().info('Auto-Initializing Robot Poses...')

        self.publish_initial_pose(self.r1_init_pub, 0.0, 0.0)
        self.publish_initial_pose(self.r2_init_pub, 1.5, 1.0)

    def publish_initial_pose(self, publisher, x, y):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.w = 1.0
        publisher.publish(msg)
        self.get_logger().info(f"Published Initial Pose for robot at ({x}, {y})")




    # Callbacks
    def coverage_cb(self, msg):
        with self.lock:
            self.coverage_data = list(msg.data)
            if self.map_info is None:
                self.map_info = msg.info

    def static_map_cb(self, msg):
        with self.lock:
            self.static_map_data = list(msg.data) 

    def r1_pose_cb(self, msg):
        #Store just x, y for easy access
        self.r1_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    def r2_pose_cb(self, msg):
        #Store just x, y for easy access
        self.r2_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        print(f"R2 Heartbeat: {self.r2_pose}")

    # Intelligent search (BFS)
    def pose_to_index(self,x,y):
        #Converts World (x,y) to Grid Index

        if self.map_info is None: return None

        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        res = self.map_info.resolution
        w = self.map_info.width

        gx = int((x - ox) / res)
        gy = int((y - oy) / res)

        if 0 <= gx < w and 0 <= gy < self.map_info.height:
            return gy *w + gx
        return None
    
    def get_nearest_unvisited_goal(self, robot_pose_xy, robot_name="Rx"):
        #Use BFS to find the closest unvisited cell.
        # This expands a circle from the robot outwards until it hits a valid 0.
        if self.coverage_data is None or self.static_map_data is None or self.map_info is None:
            self.get_logger().info(f"{robot_name}: Waiting for Data...,", throttle_duration_sec=5.0)
            return None, None
        if robot_pose_xy is None:
             # Fallback if we dont know where robot is: Pick Random
            return self.get_random_fallback()
        
        start_idx = self.pose_to_index(robot_pose_xy[0], robot_pose_xy[1])
        if start_idx is None:
            return self.get_random_fallback()
         
        width = self.map_info.width
        height = self.map_info.height
        res = self.map_info.resolution

        start_cx = start_idx % width
        start_cy = start_idx // width

        min_dist_sq = (0.8 / res) ** 2

         # BFS Initialization

        queue = deque([start_idx])
        visited_bfs = {start_idx} # keep track of what we checked IN this search

         # Limit search to prevent freezing if map is huge (e.g ., check closest 10,000 cells)
        iterations = 0
        max_iterations = 20000

        while queue:
            curr_idx = queue.popleft()
            iterations += 1
            if iterations > max_iterations:
                break # give up and try random if we cant find anything close
             
             # check candidate
            with self.lock:
                 # it it unvisited? (0 in coverage)
                 # is it free? (0 in static map)
                 # not reserved/blacklisted?

                curr_cx = curr_idx % width
                curr_cy = curr_idx // width
                dist_sq = (curr_cx - start_cx)**2 + (curr_cy - start_cy)**2

                is_far_enough = dist_sq >= min_dist_sq

                if (self.coverage_data[curr_idx] == 0 and
                    self.static_map_data[curr_idx] == 0 and
                    curr_idx not in self.reserved_indices and
                    curr_idx not in self.blacklisted):

                # safety check (inflated radius)
                # we do this last because its expensive
                    if self.is_safe_goal(curr_idx):
                        if is_far_enough:
                        # found it! reverse and return
                            self.reserved_indices.add(curr_idx)
                            return self.index_to_pose(curr_idx), curr_idx
                    
            # add neighbors

            # convert to (x,y) to handle edges correctly
            # cy = curr_idx // width
            # cx = curr_idx % width

            # check 4- connected neighbors (up, down, left, right)
            # we can do 8-connected for smoother paths, but 4 is faster

            neighbors = [
                (curr_cx+1, curr_cy), (curr_cx-1, curr_cy),
                (curr_cx, curr_cy+1), (curr_cx, curr_cy-1)
            ]

            for nx, ny in neighbors:
                if 0<=nx < width and 0 <=ny < height:
                    n_idx = ny * width + nx
                    if n_idx in visited_bfs:
                        continue

                    
                    if self.static_map_data[n_idx] != 0:
                        continue
                    visited_bfs.add(n_idx)
                    queue.append(n_idx)

        # if bfs fails then fallback to random search
        return self.get_random_fallback()
    
    def get_random_fallback(self):
        #original random login if bfs does not work as a backup

        with self.lock:
            if self.coverage_data is None or self.static_map_data is None:
                return None, None
            limit = len(self.coverage_data)

        for _ in range(200):
            idx = random.randint(0, limit - 1)
            with self.lock:
                if self.coverage_data is None or self.static_map_data is None:
                    return None, None
                if (self.coverage_data[idx] == 0 and
                    self.static_map_data[idx] == 0 and
                    idx not in self.reserved_indices and
                    idx not in self.blacklisted):

                    if self.is_safe_goal(idx):
                        self.reserved_indices.add(idx)
                        return self.index_to_pose(idx), idx
        return None, None
    
    def is_safe_goal(self, idx, buffer_radius_cells = 2):
        with self.lock:

            if self.static_map_data is None or self.map_info is None:
                return False
            
            width = self.map_info.width
            cy = idx // width
            cx = idx % width

            for dy in range(-buffer_radius_cells, buffer_radius_cells + 1):
                for dx in range(-buffer_radius_cells, buffer_radius_cells + 1):
                    nx = cx +dx
                    ny = cy + dy
                    if 0 <= nx < width and 0 <= ny < self.map_info.height:
                        n_idx = ny * width + nx
                        val = self.static_map_data[n_idx]
                        if val == 100 or val == -1:
                            return False
            return True
        
    def index_to_pose(self, idx):

        with self.lock:

            if self.map_info is None:
                return PoseStamped()

            w = self.map_info.width
            r = self.map_info.resolution
            ox = self.map_info.origin.position.x
            oy = self.map_info.origin.position.y

            y_idx = idx // w
            x_idx = idx % w

            x = (x_idx * r) + ox + (r * 0.5)
            y = (y_idx * r) + oy + (r * 0.5)

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.orientation.w = 1.0
            return goal
    
    def release_reservation(self, idx):
        if idx is None: return
        with self.lock:
            self.reserved_indices.discard(idx)


    def record_failure(self, idx):
        if idx is None: return
        with self.lock:
            self.fail_counts[idx] = self.fail_counts.get(idx, 0) + 1
            if self.fail_counts[idx] >=3:
                self.blacklisted.add(idx)


    def check_termination(self):
        with self.lock:
            if self.coverage_data is None or self.static_map_data is None:
                return False
            
            total = 0
            unvisited = 0
            for i, v in enumerate(self.static_map_data):
                if v == 0: # floor
                    total += 1
                    if self.coverage_data[i] == 0:
                        unvisited +=1

        if total ==0:
            return False
        return(1.0 - (unvisited / total)) >= 0.98
    
    def control_loop(self):
        if not self.nav_initialized:
            return
        if self.check_termination():
            self.get_logger().info("MISSION COMPLETE")
            self.timer.cancel()
            return
        
        # ROBOT 1
        if self.nav1.isTaskComplete():
            self.handle_robot_completion(self.nav1, 'r1')

            # Pass current pose to find Nearest goal
            goal, idx = self.get_nearest_unvisited_goal(self.r1_pose)

            if goal:
                self.r1_current_idx = idx
                self.get_logger().info(f"R1 -> Nearest Goal ({goal.pose.position.x:.1f}, {goal.pose.position.y:.1f})")
                self.nav1.goToPose(goal)

        # ROBOT 2
        if self.nav2.isTaskComplete():
            self.handle_robot_completion(self.nav2, 'r2')

            goal, idx = self.get_nearest_unvisited_goal(self.r2_pose)

            if goal:
                self.r2_current_idx = idx
                self.get_logger().info(f"R2 -> Nearest Goal ({goal.pose.position.x:.1f}, {goal.pose.position.y:.1f})")
                self.nav2.goToPose(goal)

    def handle_robot_completion(self, nav, robot_id):
        # Helper to clean up reservations and handle failures
        result = nav.getResult()
        current_idx = self.r1_current_idx if robot_id == 'r1' else self.r2_current_idx

        if current_idx is not None:
            self.release_reservation(current_idx)
            if result in [TaskResult.FAILED, TaskResult.CANCELED]:
                self.record_failure(current_idx)

        if robot_id == 'r1': self.r1_current_idx = None
        else:
            self.r2_current_idx = None

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotDispatcher()

    # node.get_logger().info("Waiting for Nav2 to become active...")
    # node.nav1.waitUntilNav2Active()
    # node.nav2.waitUntilNav2Active()
    # node.get_logger().info("Nav2 active. Starting dispatcher loop.")

    # node.nav_initialized = True
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()





        

        

