import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2

class RobotWalking(Node):

    def __init__(self):
        super().__init__('robot_walking')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.subscriber = self.create_subscription(PointCloud2, '/depth/points', self.update_pose, 10)
        self.vel_msg = Twist()

    def update_pose(self, msg):
        data = msg.data
        index = msg.width * msg.height // 2 + msg.width // 2
        if self.is_valid_data(data, index):
            self.vel_msg.linear.x = 1.0
        else:
            self.vel_msg.linear.x = 0.0
        self.publish_velocity()

    def is_valid_data(self, data, index):
        return len(data) != 0 and index < len(data) and data[index] in [0, 127, 128]

    def publish_velocity(self):
        self.publisher.publish(self.vel_msg)

def main():
    rclpy.init()
    node = RobotWalking()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()

