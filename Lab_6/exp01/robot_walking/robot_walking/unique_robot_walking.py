import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class RobotWalking(Node):

    def __init__(self):
        super().__init__('unique_robot_walking')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.timer = self.create_timer(2.0, self.timer_callback) 
        self.is_forward = True 

    def timer_callback(self):
        twist = Twist()

        if self.is_forward:
            twist.linear.x = 1.0
            twist.angular.z = 0.0
            self.is_forward = False
        else:
            twist.linear.x = 0.0
            twist.angular.z = math.pi / 2 
            self.is_forward = True

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    robot_walking = RobotWalking()
    rclpy.spin(robot_walking)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

