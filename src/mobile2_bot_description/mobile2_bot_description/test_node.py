import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    n = Node('test_node')
    print("✅ Node created successfully, waiting…")
    rclpy.spin(n)

if __name__ == "__main__":
    main()
