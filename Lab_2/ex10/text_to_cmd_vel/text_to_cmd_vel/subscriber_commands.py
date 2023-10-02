import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdTextSubscriber(Node):

    def __init__(self):
        super().__init__('subscriber_commands')
        self.cmd_text_subscription = self.create_subscription(
            Twist, 'turtlesim1/cmd_vel', self.cmd_text_callback, 10)

    def cmd_text_callback(self, msg):
        cmd = msg.data
        self.get_logger().info(f"Received command: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    cmd_text_subscriber = CmdTextSubscriber()
    rclpy.spin(cmd_text_subscriber)
    cmd_text_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

