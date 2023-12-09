import rclpy,sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import tf2_ros
import tf2_geometry_msgs

class RobotWalking(Node):

    def __init__(self):
        super().__init__('robor_walking')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        twist = Twist()
        twist.angular.z = 1.56  
        twist.linear.x = 3.0       
        self.publisher.publish(twist)

    

def main(args=None):
    rclpy.init(args=args)

    robot_walking = RobotWalking()

    rclpy.spin(robot_walking)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

