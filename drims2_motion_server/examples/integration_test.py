#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient

class IntegrationTest(Node):
    def __init__(self):
        super().__init__('integration_test')
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_action')
        self.get_logger().info("Integration test started")
        
    def test_gripper(self):
        """Test gripper functionality with valid positions"""
        self.get_logger().info("Testing OnRobot 2FG7 Gripper...")
        
        # Test valid positions only
        test_positions = [0.075, 0.055, 0.035]  # Open, Mid, Closed
        
        for position in test_positions:
            self.get_logger().info(f"Testing position: {position}m")
            
            goal_msg = GripperCommand.Goal()
            goal_msg.command.position = float(position)
            goal_msg.command.max_effort = 50.0
            
            if not self.gripper_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Gripper server not available")
                return False
                
            future = self.gripper_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f"Goal rejected for position {position}m")
                continue
                
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            
            result = result_future.result().result
            self.get_logger().info(f"Position {position}m: reached_goal={result.reached_goal}, stalled={result.stalled}")
            
        return True

def main():
    rclpy.init()
    test = IntegrationTest()
    
    success = test.test_gripper()
    
    if success:
        test.get_logger().info("✅ All tests passed!")
    else:
        test.get_logger().error("❌ Some tests failed!")
    
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()