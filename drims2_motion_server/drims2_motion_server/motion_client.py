import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from drims2_msgs.action import MoveToPose, MoveToJoint
from drims2_msgs.srv import AttachObject, DetachObject
from geometry_msgs.msg import PoseStamped
from control_msgs.action import GripperCommand

class MotionClient(Node):

    def __init__(self):
        super().__init__('motion_client_node')
        self.move_to_pose_client = ActionClient(self, MoveToPose, 'move_to_pose')
        self.move_to_joint_client = ActionClient(self, MoveToJoint, 'move_to_joint')
        self.gripper_client = ActionClient(self, GripperCommand, 'robotiq_action_controller/gripper_cmd')
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
        # if not self.gripper_client.wait_for_server(timeout_sec=10.0):
        #     raise RuntimeError("GripperCommand action server not available")

    def move_to_pose(self, pose: PoseStamped, cartesian_motion: bool = False):
        """API ROS-Free: Move to pose"""
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
        
        return result_future.result().result.result.val  # Success or Failure

    def move_to_joint(self, joint_positions: list[float]):
        """API ROS-Free: Move to joint positions"""
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

        return result_future.result().result.result.val  # Success or Failure

    def attach_object(self, object_id: str, target_frame_id: str):
        """API ROS-Free: Attach an object to the robot"""
        if not self.attach_object_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("AttachObject service not available")

        request = AttachObject.Request()
        request.object_id = object_id
        request.target_frame_id = target_frame_id

        future = self.attach_object_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result().success
    
    def detach_object(self, object_id: str):
        """API ROS-Free: Detach an object from the robot"""
        if not self.detach_object_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("DetachObject service not available")

        request = DetachObject.Request()
        request.object_id = object_id

        future = self.detach_object_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result().success

    def gripper_command(self, position: float, max_effort: float = 0.0):
        """API ROS-Free: Send command to Robotiq gripper"""
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("GripperCommand action server not available")

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

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