import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class RobotWalking(Node):
    def __init__(self):
        super().__init__('forward_walking')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/robot/scan',
            self.laser_callback,
            10
        )
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.has_obstacle = False

    def laser_callback(self, msg):
        # Пример: если дистанция до объекта менее 1 метра, считаем, что есть препятствие
        min_distance = min(msg.ranges)
        if min_distance < 3.0:
            self.has_obstacle = True
        else:
            self.has_obstacle = False

    def timer_callback(self):
        twist = Twist()

        if self.has_obstacle:
            twist.linear.x = 0.0   # Если есть препятствие, останавливаем линейное движение
            twist.angular.z = 0.0  # Если есть препятствие, останавливаем вращение
        else:
            twist.linear.x = 1.0   # Линейная скорость (задайте значение по вашему выбору)
            twist.angular.z = 0.0  # Угловая скорость (нулевая, чтобы робот не вращался)

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    robot_walking = RobotWalking()

    rclpy.spin(robot_walking)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

