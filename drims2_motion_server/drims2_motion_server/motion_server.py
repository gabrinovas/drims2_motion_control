import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node

from drims2_msgs.action import MoveToPose, MoveToJoint
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit.planning import MoveItPy

# from moveit.core.robot_state import RobotState


class MotionServer(Node):

    def __init__(self):
        super().__init__('motion_server_node')

        # self.declare_parameter('motion_server_config_path', '')
        self.declare_parameter('move_group_name', '')
        self.declare_parameter('target_frame', '')

        # self.get_logger().info(self.get_parameter('motion_server_config_path').get_parameter_value().string_value)
        self.move_group_name = self.get_parameter('move_group_name').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        try:     
            self.moveit_core = MoveItPy(node_name="moveit_py")
        except RuntimeError as exception:
            raise exception
        try:
            self.robot_arm = self.moveit_core.get_planning_component(self.move_group_name)
        except Exception as ex:
            raise ex

        self.move_to_pose_action_server = ActionServer(
            self,
            MoveToPose,
            "move_to_pose",
            execute_callback=self.move_to_pose_callback,
            cancel_callback=self.move_to_pose_cancel_callback,
        )
        self.move_to_pose_action_server = ActionServer(
            self,
            MoveToJoint,
            "move_to_joint",
            execute_callback=self.move_to_joint_callback,
            cancel_callback=self.move_to_joint_cancel_callback,
        )

        self.get_logger().info("Motion server is ready to receive requests")

    def move_to_pose_callback(self, goal_handle: ServerGoalHandle) -> MoveToPose.Result:
        # robot_model = self.moveit_core.get_robot_model()
        # robot_state = RobotState(robot_model)
        self.robot_arm.set_start_state_to_current_state()
        goal_pose: PoseStamped = goal_handle.request.pose_target
        self.robot_arm.set_goal_state(pose_stamped_msg = goal_pose, 
                                      pose_link=self.target_frame)
        plan_solution = self.robot_arm.plan()
        action_result = MoveToPose.Result()
        if plan_solution:
            robot_trajectory = plan_solution.trajectory
            execution_result = self.moveit_core.execute(self.move_group_name, robot_trajectory, blocking=True)
            if execution_result:     
                action_result.result.val = action_result.result.SUCCESS
            else:
                action_result.result.val = action_result.result.FAILURE

        else:
            action_result.result = plan_solution.error_code
            self.get_logger().error(f"Planning to {goal_pose} failed")
        goal_handle.succeed()

        # action_result.result = execution_result.status
        # self.get_logger().info(f'Result of the execution: {action_result.result}')
        # moveit_py.controller_manager.ExecutionStatus

        return action_result

    def move_to_pose_cancel_callback(self, goal_handle):
        pass

    def move_to_joint_cancel_callback(self, goal_handle):
        pass

    def move_to_joint_callback(self, goal_handle: ServerGoalHandle) -> MoveToJoint.Result:
        
        joints_goal = goal_handle.request.joint_target
        self.robot_arm.set_start_state_to_current_state()
        actual_robot_state = self.robot_arm.get_start_state()
        
        action_result = MoveToJoint.Result()
        if len(actual_robot_state.get_joint_group_positions(self.move_group_name)) != len(joints_goal):
            goal_handle.succeed()
            self.get_logger().error(f"Joint goal of size {len(joints_goal)} while \
                                    active joints are: {len(self.robot_arm.get_joint_group_positions(self.move_group_name))}")
            action_result.result.val = action_result.result.GOAL_STATE_INVALID
            return action_result

        actual_robot_state.set_joint_group_positions(self.move_group_name, goal_handle.request.joint_target)
        self.robot_arm.set_goal_state(robot_state=actual_robot_state)
        plan_solution = self.robot_arm.plan()
        if plan_solution:
            robot_trajectory = plan_solution.trajectory
            execution_result = self.moveit_core.execute(self.move_group_name, robot_trajectory, blocking=True)
            if execution_result:     
                action_result.result.val = action_result.result.SUCCESS
            else:
                action_result.result.val = action_result.result.FAILURE

        else:
            action_result.result = plan_solution.error_code
            self.get_logger().error(f"Planning to {joints_goal} failed")
        goal_handle.succeed()

        # action_result.result = execution_result.status
        # self.get_logger().info(f'Result of the execution: {action_result.result}')
        # moveit_py.controller_manager.ExecutionStatus

        return action_result


def main(args=None):
    rclpy.init(args=args)
    try:
        motion_server_node = MotionServer()
    except RuntimeError as e:
        rclpy.logging.get_logger("motion_server_node").error(f"MotionServer start error: {e}")
        rclpy.shutdown()
        return

    rclpy.spin(motion_server_node)

    


if __name__ == '__main__':
    main()