import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from drims2_msgs.action import MoveToPose, MoveToJoint
from geometry_msgs.msg import PoseStamped

class MotionClient(Node):

    def __init__(self):
        super().__init__('motion_client_node')
        self.move_to_pose_client = ActionClient(self, MoveToPose, 'move_to_pose')
        self.move_to_joint_client = ActionClient(self, MoveToJoint, 'move_to_joint')
        if not self.move_to_pose_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveToPose action server not available")
        if not self.move_to_joint_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveToJoint action server not available")

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

def main() -> None:
    rclpy.init()
    motion_client = MotionClient()

    # Example: move_to_pose
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = "base_link"
    pose_msg.pose.position.x = 0.4
    pose_msg.pose.position.y = 0.0
    pose_msg.pose.position.z = 0.3
    pose_msg.pose.orientation.w = 1.0

    result = motion_client.move_to_pose(pose_msg)
    print("Move to pose result:", result)

    # Example: move_to_joint
    joint_goal = [0.1, -1.2, 1.0, -1.5, 1.2, 0.5]
    result = motion_client.move_to_joint(joint_goal)
    print("Move to joint result:", result)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
