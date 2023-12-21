import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelCommand(Node):
    def __init__(self, topic_name="/cmd_vel"):
        super().__init__(node_name="velcommand")

        self.verbose = True
        self.lin_vel = None
        self.ang_vel = None

        self.set_vel([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

        self.vel_publisher = self.create_publisher(Twist, topic_name, 10)
        self.command_interval = 0.1
        self.timer = self.create_timer(self.command_interval, self.publish_vel)


    def set_vel(self, lin_vel, ang_vel):
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel

        if self.verbose:
            self.get_logger().info(f"set velocity: {self.lin_vel} {self.ang_vel}")


    def reset_vel(self):
        self.lin_vel = [0.0, 0.0, 0.0]
        self.ang_vel = [0.0, 0.0, 0.0]


    def reverse_motion(self):
        '''
        This function makes the robot move backwards.
        '''
        self.lin_vel = [-0.25, 0.0, 0.0]
        self.ang_vel = [0.0, 0.0, 0.0]


    def publish_vel(self):
        msg = Twist()

        msg.linear.x = self.lin_vel[0]
        msg.linear.y = self.lin_vel[1]
        msg.linear.z = self.lin_vel[2]

        msg.angular.x = self.ang_vel[0]
        msg.angular.y = self.ang_vel[1]
        msg.angular.z = self.ang_vel[2]

        if self.verbose:
            self.get_logger().info(f"publish velocity: {msg}")

        self.vel_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    velcmd = VelCommand()

    rclpy.spin(velcmd)

    rclpy.shutdown()


if __name__ == "__main__":
    main()