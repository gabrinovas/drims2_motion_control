import rclpy
from geometry_msgs.msg import PoseStamped
from drims2_motion_server.motion_client import MotionClient

def main() -> None:
    rclpy.init()
    
    # Test with OnRobot gripper
    motion_client = MotionClient(gripper_type='onrobot_2fg7')
    
    print("Testing OnRobot 2FG7 Gripper...")
    
    # Test gripper commands
    print("Opening gripper to 75mm...")
    success, stalled = motion_client.gripper_command(0.075, 50.0)
    print(f"Result: success={success}, stalled={stalled}")
    
    rclpy.sleep(2)
    
    print("Closing gripper to 35mm...")
    success, stalled = motion_client.gripper_command(0.035, 50.0)
    print(f"Result: success={success}, stalled={stalled}")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()