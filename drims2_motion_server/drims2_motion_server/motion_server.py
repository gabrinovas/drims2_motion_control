import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node

from drims2_msgs.action import MoveToPose, MoveToJoint
from drims2_msgs.srv import AttachObject, DetachObject
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

from pymoveit2 import MoveIt2, MoveIt2State
from threading import Thread
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.msg import MoveItErrorCodes
from drims2_move_to.drims2_move_to_py import MoveTo


class MotionServer(Node):

    def __init__(self):
        super().__init__('motion_server_node')

        self.declare_parameter('move_group_name', 'manipulator')
        self.declare_parameter('joint_names', [''])
        self.declare_parameter('base_link_name', 'base_link')
        self.declare_parameter('end_effector_name', 'tool0')
        self.declare_parameter('planner_id', 'BiTRRT')

        self.declare_parameter('cartesian_max_step', 0.0025)
        self.declare_parameter('cartesian_fraction_threshold', 0.0)
        self.declare_parameter('cartesian_jump_threshold', 0.0)
        self.declare_parameter('cartesian_avoid_collisions', True)
        self.declare_parameter("max_velocity", 0.5)
        self.declare_parameter("max_acceleration", 0.5)
        self.declare_parameter("use_move_group_action", False)
        self.declare_parameter("allowed_planning_time", 1.0)

        self.move_group_name = self.get_parameter('move_group_name').get_parameter_value().string_value
        self.end_effector_name = self.get_parameter('end_effector_name').get_parameter_value().string_value
        self.base_link_name = self.get_parameter('base_link_name').get_parameter_value().string_value
        self.joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value


        self.internal_node = Node(
            'motion_server_moveit2_internal_node', use_global_arguments=False, )
        
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        try:     
            self.moveit2 = MoveTo(
                group_name=self.move_group_name,
            )
            self.moveit2_backup = MoveIt2(
                node=self.internal_node,
                joint_names=self.joint_names,
                base_link_name=self.base_link_name,
                end_effector_name=self.end_effector_name,
                group_name=self.move_group_name,
                callback_group=self.callback_group,
                use_move_group_action=self.get_parameter('use_move_group_action').get_parameter_value().bool_value
            )
            self.moveit2_backup.planner_id = self.get_parameter('planner_id').get_parameter_value().string_value
            self.moveit2_backup.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
            self.moveit2_backup.max_acceleration = self.get_parameter('max_acceleration').get_parameter_value().double_value
            self.moveit2_backup.allowed_planning_time = self.get_parameter('allowed_planning_time').get_parameter_value().double_value


        except RuntimeError as exception:
            raise exception

        self.move_to_pose_action_server = ActionServer(
            self,
            MoveToPose,
            "move_to_pose",
            execute_callback=self.move_to_pose_callback,
            cancel_callback=self.move_to_pose_cancel_callback,
        )
        self.move_to_joint_action_server = ActionServer(
            self,
            MoveToJoint,
            "move_to_joint",
            execute_callback=self.move_to_joint_callback,
            cancel_callback=self.move_to_joint_cancel_callback,
        )
        self.attach_service = self.create_service(
            AttachObject,
            'attach_object',
            self.attach_object_callback
        )
        self.detach_service = self.create_service(
            DetachObject,
            'detach_object',
            self.detach_object_callback
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Motion server is ready to receive requests")


    def move_to_pose_callback(self, goal_handle: ServerGoalHandle) -> MoveToPose.Result:
        goal_pose: PoseStamped = goal_handle.request.pose_target
        cartesian_motion = goal_handle.request.cartesian_motion
        
        cartesian_max_step = self.get_parameter('cartesian_max_step').get_parameter_value().double_value
        cartesian_fraction_threshold = self.get_parameter('cartesian_fraction_threshold').get_parameter_value().double_value
        
        self.broadcast_pose_goal_tf(goal_pose)  # For debugging purposes show the goal pose

        pose_dict = {
            "frame_id": goal_pose.header.frame_id,
            "position": {
                "x": goal_pose.pose.position.x,
                "y": goal_pose.pose.position.y,
                "z": goal_pose.pose.position.z,
            },
            "orientation": {
                "x": goal_pose.pose.orientation.x,
                "y": goal_pose.pose.orientation.y,
                "z": goal_pose.pose.orientation.z,
                "w": goal_pose.pose.orientation.w,
            }
        }

        
        motion_result = self.moveit2.move_to_pose(
            pose=pose_dict,
            joints_names=self.joint_names,
            cartesian=cartesian_motion,
            cartesian_max_step=cartesian_max_step,
            cartesian_fraction_threshold=cartesian_fraction_threshold,
        )
        self.get_logger().info(f"Motion result: {motion_result}")

        action_result = MoveToPose.Result()
        action_result.result.val = motion_result

        goal_handle.succeed()
        return action_result

    def move_to_pose_cancel_callback(self, goal_handle):
        pass

    def move_to_joint_cancel_callback(self, goal_handle):
        pass

    def move_to_joint_callback(self, goal_handle: ServerGoalHandle) -> MoveToJoint.Result:
        
        joints_goal = goal_handle.request.joint_target
        self.get_logger().info(f"Moving to joint sfsafdas: {joints_goal}")

        motion_result = self.moveit2.move_to_joint(joints_goal, self.joint_names)
        self.get_logger().info(f"Motion result: {motion_result}")

        action_result = MoveToJoint.Result()
        action_result.result.val = motion_result

        # thread_node = Thread(target=self.test, daemon=True)
        # thread_node.start()
        # rate = self.create_rate(10)
        # self.get_logger().info("Current State: " + str(self.moveit2.query_state()))
        # while self.moveit2.query_state() != MoveIt2State.EXECUTING:
        #     self.get_logger().info(f"HEREE: {self.moveit2.query_state()}")
        #     rate.sleep()

        # # Get the future
        # self.get_logger().info("Current State: " + str(self.moveit2.query_state()))
        # future = self.moveit2.get_execution_future()
        # cancel_after_secs = 10.0

        # # Cancel the goal
        # if cancel_after_secs > 0.0:
        #     # Sleep for the specified time
        #     sleep_time = self.create_rate(cancel_after_secs)
        #     sleep_time.sleep()
        #     # Cancel the goal
        #     self.get_logger().info("Cancelling goal")
        #     self.moveit2.cancel_execution()

        # # Wait until the future is done
        # while not future.done():
        #     rate.sleep()
        # # thread_node.join()
        # # Print the result
        # self.get_logger().info("Result status: " + str(future.result().status))
        # self.get_logger().info("Result error code: " + str(future.result().result.error_code))

        
        goal_handle.succeed()
        return action_result

    def broadcast_pose_goal_tf(self, pose_goal):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = pose_goal.header.frame_id
        t.child_frame_id = "pose_goal_frame" 

        t.transform.translation.x = pose_goal.pose.position.x
        t.transform.translation.y = pose_goal.pose.position.y
        t.transform.translation.z = pose_goal.pose.position.z

        t.transform.rotation = pose_goal.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def attach_object_callback(self, request, response):
        self.get_logger().info(f"Attaching object '{request.object_id}' to frame '{request.target_frame_id}'")
        self.moveit2_backup.attach_collision_object(id=request.object_id,
                                             link_name=request.target_frame_id,
                                             touch_links=[request.target_frame_id])
        response.success = True        
        return response


    def detach_object_callback(self, request, response):
        self.get_logger().info(f"Detaching object '{request.object_id}'")
        self.moveit2_backup.detach_collision_object(request.object_id)
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    try:
        motion_server_node = MotionServer()
    except RuntimeError as e:
        rclpy.logging.get_logger("motion_server_node").error(f"MotionServer start error: {e}")
        rclpy.shutdown()
        return
    executor.add_node(motion_server_node)
    executor.spin()

if __name__ == '__main__':
    main()
