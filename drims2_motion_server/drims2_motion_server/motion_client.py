import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from typing import Tuple

from drims2_msgs.action import MoveToPose, MoveToJoint
from drims2_msgs.srv import AttachObject, DetachObject

from geometry_msgs.msg import PoseStamped
from control_msgs.action import GripperCommand
from moveit_msgs.msg import MoveItErrorCodes

class MotionClient(Node):
    """ROS 2 Client for controlling robot motion and gripper.

    This class provides a simplified interface for sending commands
    to the robot via action servers and services. Supported operations:
    - Move to a target pose
    - Move to joint configurations
    - Attach/detach objects
    - Move the gripper (supports both Robotiq and OnRobot 2FG7)
    """


    def __init__(self,
                 move_to_pose_action_name:str ='move_to_pose',
                 move_to_joint_action_name:str ='move_to_joint',
                 gripper_type:str ='onrobot_2fg7'):
        super().__init__('motion_client_node', use_global_arguments=False)
        """Initialize the MotionClient.

        Args:
            move_to_pose_action_name (str): Action server name for moving to a pose.
            move_to_joint_action_name (str): Action server name for moving to joint positions.
            gripper_type (str): Type of gripper - 'robotiq' or 'onrobot_2fg7'

        Raises:
            RuntimeError: If one of the required action servers or services is not available.
        """


        self.declare_parameter('move_to_pose_action_name', move_to_pose_action_name)
        self.declare_parameter('move_to_joint_action_name', move_to_joint_action_name)
        self.declare_parameter('gripper_type', gripper_type)

        move_to_pose_action_name = self.get_parameter('move_to_pose_action_name').get_parameter_value().string_value
        move_to_joint_action_name = self.get_parameter('move_to_joint_action_name').get_parameter_value().string_value
        gripper_type = self.get_parameter('gripper_type').get_parameter_value().string_value

        # Set gripper action name based on type
        if gripper_type == "robotiq":
            gripper_action_name = "/robotiq_action_controller/gripper_cmd"
            self.get_logger().info("Using Robotiq gripper configuration")
        elif gripper_type == "onrobot_2fg7":
            gripper_action_name = "/gripper_action"
            self.get_logger().info("Using OnRobot 2FG7 gripper configuration")
        else:
            self.get_logger().warn(f"Unknown gripper type: {gripper_type}, defaulting to OnRobot 2FG7")
            gripper_action_name = "/gripper_action"
            gripper_type = "onrobot_2fg7"

        self.gripper_type = gripper_type

        self.move_to_pose_client = ActionClient(self, MoveToPose, move_to_pose_action_name)
        self.move_to_joint_client = ActionClient(self, MoveToJoint, move_to_joint_action_name)
        self.gripper_client = ActionClient(self, GripperCommand, gripper_action_name)
        self.attach_object_client = self.create_client(AttachObject, 'attach_object')
        self.detach_object_client = self.create_client(DetachObject, 'detach_object')

        if not self.move_to_pose_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveToPose action server not available")
        if not self.move_to_joint_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveToJoint action server not available")
        if not self.attach_object_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("AttachObject service not available")
        if not self.detach_object_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("DetachObject service not available")
        if not self.gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn(f"GripperCommand action server not available at {gripper_action_name}")
            # Don't raise error as gripper might not be critical for all operations

    def move_to_pose(self, pose: PoseStamped, 
                     cartesian_motion: bool = False) -> MoveItErrorCodes:
        """Move the robot to a target pose.

        Args:
            pose (PoseStamped): Target pose for the robot.
            cartesian_motion (bool, optional): If True, uses Cartesian trajectories. Defaults to False.

        Returns:
            MoveItErrorCodes: Result code returned by the motion planner.

        Raises:
            RuntimeError: If the action server is not available or the goal was rejected.
        """
        if not self.move_to_pose_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("MoveToPose action server not available")

        goal_msg = MoveToPose.Goal()
        goal_msg.pose_target = pose
        goal_msg.cartesian_motion = cartesian_motion

        future = self.move_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Goal to move_to_pose was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        return result_future.result().result.result

    def move_to_joint(self, joint_positions: list[float]) -> MoveItErrorCodes:
        """Move the robot to a specific joint configuration.

        Args:
            joint_positions (list[float]): List of target joint values.

        Returns:
            MoveItErrorCodes: Result code returned by the motion planner.

        Raises:
            RuntimeError: If the action server is not available or the goal was rejected.
        """
        if not self.move_to_joint_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("MoveToJoint action server not available")

        goal_msg = MoveToJoint.Goal()
        goal_msg.joint_target = joint_positions

        future = self.move_to_joint_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Goal to move_to_joint was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        return result_future.result().result.result

    def attach_object(self, object_id: str, target_frame_id: str) -> bool:
        """Attach an object to the robot (e.g., to the gripper).

        Args:
            object_id (str): ID of the object in the MoveIt scene.
            target_frame_id (str): Frame of the robot to which the object should be attached.

        Returns:
            bool: True if the operation succeeded, False otherwise.

        Raises:
            RuntimeError: If the service is not available.
        """
        if not self.attach_object_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("AttachObject service not available")

        request = AttachObject.Request()
        request.object_id = object_id
        request.target_frame_id = target_frame_id

        future = self.attach_object_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result().success
    
    def detach_object(self, object_id: str) -> bool:
        """Detach an object from the robot.

        Args:
            object_id (str): ID of the object to detach.

        Returns:
            bool: True if the operation succeeded, False otherwise.

        Raises:
            RuntimeError: If the service is not available.
        """
        if not self.detach_object_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("DetachObject service not available")

        request = DetachObject.Request()
        request.object_id = object_id

        future = self.detach_object_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result().success

    def gripper_command(self, 
                        position: float, 
                        max_effort: float = 0.0) -> Tuple[bool, bool]:
        """Send a command to the gripper, i.e, move it to a specific position.

        Args:
            position (float): Target position of the gripper.
                - For Robotiq: position in meters (0.0 to 0.79)
                - For OnRobot 2FG7: total opening in meters (0.035 to 0.075)
            max_effort (float, optional): Maximum force applied. 
                - For Robotiq: typically 0.0 to 1.0
                - For OnRobot 2FG7: force in Newtons (0.0 to 100.0)

        Returns:
            Tuple[bool, bool]:
                - reached_goal (bool): True if the gripper reached the target position.
                - stalled (bool): True if the gripper stalled during execution.

        Raises:
            RuntimeError: If the action server is not available or the goal was rejected.
        """
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("GripperCommand action server not available")

        goal_msg = GripperCommand.Goal()
        
        # Handle different gripper types
        if self.gripper_type == "onrobot_2fg7":
            # OnRobot 2FG7: Convert total opening to finger position
            # Total opening range: 0.035m (closed) to 0.075m (open)
            # Each finger moves 0.0 to 0.020m from center
            total_opening = max(0.035, min(0.075, position))
            finger_position = (total_opening - 0.035) / 2.0  # Convert to single finger movement
            goal_msg.command.position = float(finger_position)
            goal_msg.command.max_effort = float(max_effort)
            self.get_logger().info(f"OnRobot 2FG7 command - Total opening: {total_opening:.3f}m, Finger position: {finger_position:.3f}m, Force: {max_effort:.1f}N")
        else:
            # Robotiq: Use position directly (0.0 to 0.79)
            goal_msg.command.position = float(position)
            goal_msg.command.max_effort = float(max_effort)
            self.get_logger().info(f"Robotiq command - Position: {position:.3f}m, Max effort: {max_effort:.1f}")

        future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Goal to gripper_command was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        reached_goal = result_future.result().result.reached_goal
        stalled = result_future.result().result.stalled

        return reached_goal, stalled