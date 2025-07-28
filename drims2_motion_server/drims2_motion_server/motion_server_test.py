import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node

from drims2_msgs.action import MoveToPose, MoveToJoint
from drims2_msgs.srv import AttachObject, DetachObject
from geometry_msgs.msg import PoseStamped, TransformStamped
from moveit_msgs.msg import MoveItErrorCodes, CollisionObject, AttachedCollisionObject
from moveit.planning import MoveItPy, PlanRequestParameters
from tf2_ros import TransformBroadcaster
from drims2_motion_server.drims2_utils import CartesianPlannerParams

from pymoveit2 import MoveIt2, MoveIt2State
from threading import Thread
from rclpy.executors import MultiThreadedExecutor
class MotionServer():

    def __init__(self, move_group_name, joint_names, base_link_name, end_effector_name, planner_id):
        # super().__init__('motion_server_node')
        try:
            rclpy.init()
        except Exception as e:
            if not rclpy.ok():
                return
        self.node = Node("motion_server_node")
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        try:     
            self.moveit2 = MoveIt2(
                node=self.node,
                joint_names=joint_names,
                base_link_name=base_link_name,
                end_effector_name=end_effector_name,
                group_name=move_group_name,
                callback_group=self.callback_group,
            )

            # Avvia il thread DOPO aver creato tutti i callback

            # self.executor_thread = Thread(target=self.spin_internal_executor, daemon=True)
            # self.executor_thread.start()

        except RuntimeError as exception:
            raise exception
        self.executor = rclpy.executors.MultiThreadedExecutor(2)
        self.executor.add_node(self.node)
        self.executor_thread = Thread(target=self.executor.spin, daemon=True, args=())
        self.executor_thread.start()
        # self.move_to_pose_action_server = ActionServer(
        #     self,
        #     MoveToPose,
        #     "move_to_pose",
        #     execute_callback=self.move_to_pose_callback,
        #     cancel_callback=self.move_to_pose_cancel_callback,
        # )
        # self.move_to_pose_action_server = ActionServer(
        #     self,
        #     MoveToJoint,
        #     "move_to_joint",
        #     execute_callback=self.move_to_joint_callback,
        #     cancel_callback=self.move_to_joint_cancel_callback,
        # )
        # self.attach_service = self.create_service(
        #     AttachObject,
        #     'attach_object',
        #     self.attach_object_callback
        # )
        # self.detach_service = self.create_service(
        #     DetachObject,
        #     'detach_object',
        #     self.detach_object_callback
        # )
        # # self.executor_thread = Thread(target=self.internal_executor.spin, daemon=True, args=())
        # # self.executor_thread.start()

        # self.tf_broadcaster = TransformBroadcaster(self)

        # self.get_logger().info("Motion server is ready to receive requests")


    def move_to_joint_callback(self, joint_goal):
        
        joints_goal = joint_goal
        # self.get_logger().info(f"Moving to joint sfsafdas: {joints_goal}")


        # self.internal_node.declare_parameter('synchronous', True)
        self.moveit2.move_to_configuration(joints_goal)
        
        # threading.Thread(target=self.moveit2.move_to_configuration(joints_goal)).start()
        # self.moveit2.wait_until_executed()
        print("Current State: " + str(self.moveit2.query_state()))
        rate = self.internal_node.create_rate(10)
        while self.moveit2.query_state() != MoveIt2State.EXECUTING:
            rate.sleep()

        # Get the future
        print("Current State: " + str(self.moveit2.query_state()))
        future = self.moveit2.get_execution_future()
        cancel_after_secs = 10.0
        # Cancel the goal
        if cancel_after_secs > 0.0:
            # Sleep for the specified time
            sleep_time = self.internal_node.create_rate(cancel_after_secs)
            sleep_time.sleep()
            # Cancel the goal
            print("Cancelling goal")
            self.moveit2.cancel_execution()

        # Wait until the future is done
        while not future.done():
            rate.sleep()

        # Print the result
        print("Result status: " + str(future.result().status))
        print("Result error code: " + str(future.result().result.error_code))




def main(args=None):
    rclpy.init(args=args)
        # # Example: move_to_joint
    joint_goal = [
        -0.16410449298697685,    # shoulder_pan_joint
        -1.3820258857637684,     # shoulder_lift_joint
        1.6139817698694139,      # elbow_joint
        -1.8017236579269869,     # wrist_1_joint
        -1.5701870879802997,     # wrist_2_joint
        -0.16411033649582998,    # wrist_3_joint
    ]
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
    try:
        motion_server_node = MotionServer('manipulator',
                                          ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
                                          'base_link', 'tool0', 'OMPL')
    except RuntimeError as e:
        rclpy.logging.get_logger("motion_server_node").error(f"MotionServer start error: {e}")
        rclpy.shutdown()
        return
    motion_server_node.move_to_joint_callback(joint_goal)
    # executor.add_node(motion_server_node)
    executor.spin()

if __name__ == '__main__':
    main()