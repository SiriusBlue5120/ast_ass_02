import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserScanner(Node):
    def __init__(self):
        super.__init__(node_name="lasermon")

        self.laser_subscriber = self.create_subscription(
            LaserScan, "/scan",
            self.get_scan, 10
        )

    
    def get_scan_ranges(self, scan):
        scan_ranges = scan.ranges

        self.get_logger().info(f"/scan: recieved 'ranges' of length {len(scan_ranges)}")

        return scan_ranges
    

def main(args=None):
    rclpy.init(args=args)

    laserScanner = LaserScanner()

    rclpy.spin(laserScanner)

    rclpy.shutdown()


if __name__ == "__main__":
    main()