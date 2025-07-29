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

class MotionServer(Node):

    def __init__(self):
        super().__init__('motion_server_node')

        # self.declare_parameter('motion_server_config_path', '')
        self.declare_parameter('move_group_name', 'manipulator')
        self.declare_parameter('joint_names', [''])
        self.declare_parameter('base_link_name', 'base_link')
        self.declare_parameter('end_effector_name', 'tool0')
        self.declare_parameter('planner_id', 'RRTConnectkConfigDefault')

        self.declare_parameter('cartesian_max_step', 0.0025)
        self.declare_parameter('cartesian_fraction_threshold', 0.0)
        self.declare_parameter('cartesian_jump_threshold', 0.0)
        self.declare_parameter('cartesian_avoid_collisions', True)
        self.declare_parameter("max_velocity", 0.5)
        self.declare_parameter("max_acceleration", 0.5)

        # self.get_logger().info(self.get_parameter('motion_server_config_path').get_parameter_value().string_value)
        self.move_group_name = self.get_parameter('move_group_name').get_parameter_value().string_value
        self.end_effector_name = self.get_parameter('end_effector_name').get_parameter_value().string_value
        self.base_link_name = self.get_parameter('base_link_name').get_parameter_value().string_value
        self.joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        self.target_frame = self.get_parameter('base_link_name').get_parameter_value().string_value


        self.internal_node = Node(
            'motion_server_moveit2_internal_node', use_global_arguments=False, )
        
        # self.internal_executor = rclpy.executors.MultiThreadedExecutor(2)
        # self.internal_executor.add_node(self.internal_node)
        # self.internal_node.declare_parameter('synchronous', False)

        callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        try:     
            self.moveit2 = MoveIt2(
                node=self.internal_node,
                joint_names=self.joint_names,
                base_link_name=self.base_link_name,
                end_effector_name=self.end_effector_name,
                group_name=self.move_group_name,
                callback_group=callback_group,
            )
            self.moveit2.planner_id = self.get_parameter('planner_id').get_parameter_value().string_value
            self.moveit2.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
            self.moveit2.max_acceleration = self.get_parameter('max_acceleration').get_parameter_value().double_value

            # self.executor_thread = Thread(target=self.spin_internal_executor, daemon=True)
            # self.executor_thread.start()

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
        # self.executor_thread = Thread(target=self.internal_executor.spin, daemon=True, args=())
        # self.executor_thread.start()

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Motion server is ready to receive requests")


    def move_to_pose_callback(self, goal_handle: ServerGoalHandle) -> MoveToPose.Result:
        goal_pose: PoseStamped = goal_handle.request.pose_target
        cartesian_motion = goal_handle.request.cartesian_motion
        
        cartesian_max_step = self.get_parameter('cartesian_max_step').get_parameter_value().double_value
        cartesian_fraction_threshold = self.get_parameter('cartesian_fraction_threshold').get_parameter_value().double_value

        self.moveit2.move_to_pose(
            pose=goal_pose,
            cartesian=cartesian_motion,
            cartesian_max_step=cartesian_max_step,
            cartesian_fraction_threshold=cartesian_fraction_threshold,
        )
        self.moveit2.wait_until_executed()
        # self.robot_arm.set_goal_state(pose_stamped_msg = goal_pose, 
        #                               pose_link=self.target_frame)
        # self.broadcast_pose_goal_tf(goal_pose)

        # if cartesian_motion:
        #     self.get_logger().info(f"Move to Pose with linear cartesian motion.")
        #     plan_request_parameters = CartesianPlannerParams.get_plan_request_parameters(self,self.moveit_core)
        #     plan_solution = self.robot_arm.plan(plan_request_parameters)
        #     self.get_logger().info(f"Plan solution")
        # else:
        #     self.get_logger().info(f"Move to Pose with joint motion.")
        #     plan_solution = self.robot_arm.plan()
        #     self.get_logger().info(f"Plan solution")


        # action_result = MoveToPose.Result()
        # if plan_solution:
        #     robot_trajectory = plan_solution.trajectory
        #     self.get_logger().info(f"Trj pre executed")
        #     execution_result = self.moveit_core.execute(self.move_group_name, robot_trajectory, blocking=True)
        #     self.get_logger().info(f"Trj executed")

        #     if execution_result:     
        #         action_result.result.val = action_result.result.SUCCESS
        #     else:
        #         action_result.result.val = action_result.result.FAILURE

        # else:
        #     action_result.result = plan_solution.error_code
        #     self.get_logger().error(f"Planning to {goal_pose} failed")
        goal_handle.succeed()

        action_result = MoveToPose.Result()
        action_result.result.val = action_result.result.SUCCESS

        return action_result

    def move_to_pose_cancel_callback(self, goal_handle):
        pass

    def move_to_joint_cancel_callback(self, goal_handle):
        pass

    def move_to_joint_callback(self, goal_handle: ServerGoalHandle) -> MoveToJoint.Result:
        
        joints_goal = goal_handle.request.joint_target
        self.get_logger().info(f"Moving to joint sfsafdas: {joints_goal}")

        self.moveit2.move_to_configuration(joints_goal)
        self.moveit2.wait_until_executed()
        
        # # thread_node = Thread(target=self.internal_executor.spin, daemon=True)
        # # thread_node.start()
        # self.get_logger().info("Current State: " + str(self.moveit2.query_state()))
        # rate = self.create_rate(10)
        # while self.moveit2.query_state() != MoveIt2State.EXECUTING:
        #     rate.sleep()

        # # Get the future
        # self.get_logger().info("Current State: " + str(self.moveit2.query_state()))
        # future = self.moveit2.get_execution_future()
        # cancel_after_secs = 10.0

        # # Cancel the goal
        # if cancel_after_secs > 0.0:
        #     # Sleep for the specified time
        #     sleep_time = self.internal_node.create_rate(cancel_after_secs)
        #     sleep_time.sleep()
        #     # Cancel the goal
        #     self.get_logger().info("Cancelling goal")
        #     self.moveit2.cancel_execution()

        # # Wait until the future is done
        # while not future.done():
        #     rate.sleep()

        # Print the result
        # self.get_logger().info("Result status: " + str(future.result().status))
        # self.get_logger().info("Result error code: " + str(future.result().result.error_code))

        goal_handle.succeed()
        action_result = MoveToJoint.Result()
        action_result.result.val = action_result.result.SUCCESS
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
        self.moveit2.attach_collision_object(id=request.object_id,
                                             link_name=request.target_frame_id,
                                             touch_links=[request.target_frame_id])        
        return response


    def detach_object_callback(self, request, response):
        self.get_logger().info(f"Detaching object '{request.object_id}'")
        self.moveit2.detach_collision_object(request.object_id)
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