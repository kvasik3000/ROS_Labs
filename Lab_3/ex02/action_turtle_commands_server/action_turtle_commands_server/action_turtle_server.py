import time
import math
import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from action_turtle_commands.action import MessageTurtleCommands

class MessageTurtleActionServer(Node):

    def __init__(self):
        super().__init__('message_turtle_action_server')
        # Создание сервера действия
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'messageturtle',
            self.execute_callback)
        # Создание издателя для управления движением черепахи
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 1)

    def execute_callback(self, goal_handle):
        # Обработка цели действия
        self.get_logger().info('Executing goal...')
        command = goal_handle.request.command
        s = goal_handle.request.s
        angle = goal_handle.request.angle
        message = Twist()
        self.publisher_.publish(message)

        feedback_msg = MessageTurtleCommands.Feedback()
        feedback_msg.odom = 0

        if command == "forward":
            # Если команда "forward", двигаемся вперед
            self.get_logger().info('Goal: forward, {0} meters'.format(goal_handle.request.s))
            for i in range(goal_handle.request.s):
                feedback_msg.odom += 1
                message.linear.x += 1.0
                self.publisher_.publish(message)
                time.sleep(1)
                self.get_logger().info('Feedback: {0} meters'.format(feedback_msg.odom))
                goal_handle.publish_feedback(feedback_msg)
                message.linear.x -= 1.0
                time.sleep(1)
        if command == "turn_right":
            # Если команда "turn_right", поворачиваем направо
            self.get_logger().info('Goal: turn right, {0} degrees'.format(goal_handle.request.angle))
            message.angular.z = - angle * np.pi / 180.0  # Преобразование градусов в радианы
            self.publisher_.publish(message)
            time.sleep(1)
            self.get_logger().info('Feedback: {0} meters'.format(feedback_msg.odom))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        # Отмечаем успешное завершение цели
        goal_handle.succeed()
        result = MessageTurtleCommands.Result()
        result.result = True
        return result

def main(args=None):
    rclpy.init(args=args)
    message_turtle_action_server = MessageTurtleActionServer()
    rclpy.spin(message_turtle_action_server)

if __name__ == '__main__':
    main()

