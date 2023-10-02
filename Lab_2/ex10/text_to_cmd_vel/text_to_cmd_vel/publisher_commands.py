import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TextToCmdVelNode(Node):

    def __init__(self):
        super().__init__('publisher_commands')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.cmd_vel_subscriber_ = self.create_subscription(String,'cmd_text',self.cmd_text_callback,10)
  #      timer_period = 0.5  # seconds
   #     self.timer_= self.create_timer(timer_period, self.cmd_text_callback)
        

    def cmd_text_callback(self,msg):
        cmd = msg.data
        twist = Twist()

        if cmd == 'turn_right':
            twist.angular.z = -1.5
        elif cmd == 'turn_left':
            twist.angular.z = 1.5
        elif cmd == 'move_forward':
            twist.linear.x = 1.0
        elif cmd == 'move_backward':
            twist.linear.x = -1.0

        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    text_to_cmd_vel_node = TextToCmdVelNode()
    rclpy.spin(text_to_cmd_vel_node)
    text_to_cmd_vel_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
