import rclpy
from geometry_msgs.msg import PoseStamped
from drims2_motion_server.motion_client import MotionClient

def main() -> None:
    rclpy.init()
    motion_client = MotionClient()

    # Example: move_to_pose
    # pose_msg = PoseStamped()
    # pose_msg.header.frame_id = "base_link"
    # pose_msg.pose.position.x = 0.4
    # pose_msg.pose.position.y = 0.0
    # pose_msg.pose.position.z = 0.3
    # pose_msg.pose.orientation.w = 1.0

    # result = motion_client.move_to_pose(pose_msg)
    # print("Move to pose result:", result)

    # # Example: move_to_joint
    # joint_goal = [0.1, -1.2, 1.0, -1.5, 1.2, 0.5]
    # result = motion_client.move_to_joint(joint_goal)
    # print("Move to joint result:", result)

    # # Example: attach_object
    # motion_client.attach_object("dice", "ur10e_tool0")
    # result = motion_client.move_to_pose(pose_msg)
    # motion_client.detach_object("dice")
    motion_client.gripper_command(0.4)  # Open gripper
    rclpy.shutdown()

if __name__ == '__main__':
    main()
