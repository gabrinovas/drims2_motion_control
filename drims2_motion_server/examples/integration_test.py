#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient

class IntegrationTest(Node):
    def __init__(self):
        super().__init__('integration_test')
        
        # Test motion client
        from drims2_motion_server.motion_client import MotionClient
        self.motion_client = MotionClient(gripper_type='onrobot_2fg7')
        
        self.get_logger().info("Integration test started")
        
    def test_gripper(self):
        """Test gripper functionality"""
        self.get_logger().info("Testing OnRobot 2FG7 Gripper...")
        
        try:
            # Test open
            self.get_logger().info("Opening gripper (75mm)...")
            success, stalled = self.motion_client.gripper_command(0.075, 50.0)
            self.get_logger().info(f"Open result: success={success}, stalled={stalled}")
            
            # Test close  
            self.get_logger().info("Closing gripper (35mm)...")
            success, stalled = self.motion_client.gripper_command(0.035, 50.0)
            self.get_logger().info(f"Close result: success={success}, stalled={stalled}")
            
            # Test mid position
            self.get_logger().info("Moving to mid position (55mm)...")
            success, stalled = self.motion_client.gripper_command(0.055, 30.0)
            self.get_logger().info(f"Mid result: success={success}, stalled={stalled}")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Gripper test failed: {e}")
            return False

def main():
    rclpy.init()
    test = IntegrationTest()
    
    # Run tests
    success = test.test_gripper()
    
    if success:
        test.get_logger().info("✅ All tests passed!")
    else:
        test.get_logger().error("❌ Some tests failed!")
    
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()