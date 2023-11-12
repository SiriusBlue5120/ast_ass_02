import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


class LaserScanSubscribe(Node):

    def __init__(self):

        super().__init__(node_name="laser_scan_sub")
        self.my_subscription = self.create_subscription(LaserScan(),"/scan",self.my_callback, 10)
        self.msg = LaserScan()

    def my_callback(self,msg) -> None:
        self.get_logger().info(f"Message received:{msg.data}")
        self.msg = msg

    def getLaserScanData(self):
        return self.msg.ranges
    
def main(agrs = None) -> None:
    rclpy.init(args=agrs)
    node = LaserScanSubscribe()
    rclpy.spin(node)
    node.destroy_node() #clean memory this is optional
    rclpy.shutdown()

if __name__ == "__main__":
    main()

