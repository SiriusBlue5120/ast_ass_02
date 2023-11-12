import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32



class BatteryVoltage(Node):

    def __init__(self):

        super().__init__(node_name="battery_voltage_sub")
        self.my_subscription = self.create_subscription(Float32(),"/battery_voltage",self.my_callback, 10)
       

    def my_callback(self,msg) -> None:
        self.get_logger().info(f"Voltage received:{msg.data}")
        self.msg = msg

    def getBatteryVoltage(self):
        return self.msg.data
    
def main(agrs = None) -> None:
    rclpy.init(args=agrs)
    node = BatteryVoltage()
    rclpy.spin(node)
    node.destroy_node() #clean memory this is optional
    rclpy.shutdown()

if __name__ == "__main__":
    main()

