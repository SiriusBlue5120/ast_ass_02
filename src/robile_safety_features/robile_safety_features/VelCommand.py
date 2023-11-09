import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelCommand(Node):
    def __init__(self, topic_name="/cmd_vel"):
        super().__init__(node_name="velcommand")

        self.set_vel([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

        self.vel_publisher = self.create_publisher(Twist, topic_name, 10)
        self.command_interval = 0.1
        self.timer = self.create_timer(self.command_interval, self.publish_vel)


    def set_vel(self, lin_vel, ang_vel):
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel


    def publish_vel(self):
        msg = Twist()

        msg.linear.x = self.lin_vel[0]
        msg.linear.y = self.lin_vel[1]
        msg.linear.z = self.lin_vel[2]

        msg.angular.x = self.ang_vel[0]
        msg.angular.y = self.ang_vel[1]
        msg.angular.z = self.ang_vel[2]

        self.vel_publisher.publish(msg)