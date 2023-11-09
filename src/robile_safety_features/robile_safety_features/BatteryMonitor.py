import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float32
from sensor_msgs.msg import LaserScan

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__(node_name="battmon")

        self.battery_level = 100.0

        self.battery_subscriber = self.create_subscription(
            Float32(), "/battery_voltage",
            self.get_battery, 10
        )

    
    def get_battery(self, battery_msg):
        self.battery_level = battery_msg.data

        self.get_logger().info(f"/battery_voltage {self.battery_level}")
