import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float32

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__(node_name="battmon")

        self.battery_subscriber = self.create_subscription(
            Float32, "/battery_voltage",
            self.get_battery, 10
        )

    
    def get_battery(self, battery_msg):
        battery_level = battery_msg.data

        self.get_logger().info(f"/battery_voltage: {battery_level}")

        return battery_level
