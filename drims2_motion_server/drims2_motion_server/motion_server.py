import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from drims2_msgs.action import MoveTo
from action_tutorials_interfaces.action import Fibonacci


class MotionServer(Node):

    def __init__(self):
        super().__init__('motion_server_node')

        self.declare_parameter('move_group_name', 'manipulator')
        move_group_name = self.get_parameter('move_group_name').get_parameter_value().string_value

        # self.moveit =     
        locobot = MoveItPy(node_name="moveit_py")
        robot_arm = locobot.get_planning_component(move_group_name)

        self.action_server = ActionServer(
            self,
            MoveTo,
            "move_to",
            execute_callback=self.move_to_callback,
            cancel_callback=self.move_to_cancel_callback,
        )
        self.declare_parameter('move_group', '')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()