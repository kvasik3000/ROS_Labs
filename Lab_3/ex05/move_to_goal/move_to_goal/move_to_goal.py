import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import sys

class MoveToGoalNode(Node):

    def __init__(self):
        super().__init__('move_to_goal')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.move_to_goal)
        self.current_pose = None
        self.target_x = float(sys.argv[1])
        self.target_y = float(sys.argv[2])
        self.target_theta = float(sys.argv[3])
        self.reached_goal = False

    def pose_callback(self, msg):
        self.current_pose = msg

    def move_to_goal(self):
        if self.current_pose is not None and not self.reached_goal:
            
            angle = math.atan2(self.target_y - self.current_pose.y, self.target_x - self.current_pose.x)

            
            linear_vel = 0.5 * math.sqrt((self.target_x - self.current_pose.x) ** 2 + (self.target_y - self.current_pose.y) ** 2)

            
            angular_vel = 2.0 * (angle - self.current_pose.theta)

            
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = linear_vel
            cmd_vel_msg.angular.z = angular_vel

            self.cmd_vel_publisher.publish(cmd_vel_msg)

            
            if linear_vel < 0.1:
                self.reached_goal = True
                self.get_logger().info("Reached the goal")
                quit()
                 

def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__':
    if len(sys.argv) != 4:
        print("Usage: ros2 run move_to_goal move_to_goal <target_x> <target_y> <target_theta>")
    else:
        main()

