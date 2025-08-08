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
from moveit_msgs.srv import GetPositionIK

class MotionServer(Node):

    def __init__(self):
        super().__init__('motion_server_node')

        self.declare_parameter('move_group_name', 'manipulator')
        self.declare_parameter('joint_names', [''])
        self.declare_parameter('base_link_name', 'base_link')
        self.declare_parameter('end_effector_name', 'tool0')
        self.declare_parameter('planner_id', 'BiTRRT')

        self.declare_parameter('cartesian_max_step', 0.005)
        self.declare_parameter('cartesian_fraction_threshold', 0.001)
        self.declare_parameter('cartesian_jump_threshold', 0.001)
        self.declare_parameter('cartesian_avoid_collisions', True)
        self.declare_parameter("max_velocity", 0.5)
        self.declare_parameter("max_acceleration", 0.5)
        self.declare_parameter("use_move_group_action", False)
        self.declare_parameter("allowed_planning_time", 2.0)
        self.declare_parameter("tolerance_position", 0.001)
        self.declare_parameter("tolerance_orientation", 0.001)

        self.move_group_name = self.get_parameter('move_group_name').get_parameter_value().string_value
        self.end_effector_name = self.get_parameter('end_effector_name').get_parameter_value().string_value
        self.base_link_name = self.get_parameter('base_link_name').get_parameter_value().string_value
        self.joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value


        self.internal_node = Node(
            'motion_server_moveit2_internal_node', use_global_arguments=False, )
        
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        try:     
            self.moveit2 = MoveIt2(
                node=self.internal_node,
                joint_names=self.joint_names,
                base_link_name=self.base_link_name,
                end_effector_name=self.end_effector_name,
                group_name=self.move_group_name,
                callback_group=self.callback_group,
                use_move_group_action=self.get_parameter('use_move_group_action').get_parameter_value().bool_value
            )
            self.moveit2.planner_id = self.get_parameter('planner_id').get_parameter_value().string_value
            self.moveit2.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
            self.moveit2.max_acceleration = self.get_parameter('max_acceleration').get_parameter_value().double_value
            self.moveit2.allowed_planning_time = self.get_parameter('allowed_planning_time').get_parameter_value().double_value
        
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
        self.compute_ik_client = self.create_client(
            GetPositionIK,
            'compute_ik',
            callback_group=self.callback_group
        )
        while not self.compute_ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for 'compute_ik' service...")


        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Motion server is ready to receive requests")


    def move_to_pose_callback(self, goal_handle: ServerGoalHandle) -> MoveToPose.Result:
        goal_pose: PoseStamped = goal_handle.request.pose_target
        cartesian_motion = goal_handle.request.cartesian_motion
        
        cartesian_max_step = self.get_parameter('cartesian_max_step').get_parameter_value().double_value
        cartesian_fraction_threshold = self.get_parameter('cartesian_fraction_threshold').get_parameter_value().double_value
        
        self.broadcast_pose_goal_tf(goal_pose)  # For debugging purposes show the goal pose

        if cartesian_motion:
            cartesian_max_step = self.get_parameter('cartesian_max_step').get_parameter_value().double_value
            cartesian_fraction_threshold = self.get_parameter('cartesian_fraction_threshold').get_parameter_value().double_value

            self.moveit2.move_to_pose(
                pose=goal_pose,
                cartesian=True,
                cartesian_max_step=cartesian_max_step,
                cartesian_fraction_threshold=cartesian_fraction_threshold,
                tolerance_position=self.get_parameter('tolerance_position').get_parameter_value().double_value,
                tolerance_orientation=self.get_parameter('tolerance_orientation').get_parameter_value().double_value
            )
            partial_result = self.moveit2.wait_until_executed()
            motion_result = self.moveit2.get_last_execution_error_code()
            self.get_logger().info(f"Partial result: {partial_result}")
            self.get_logger().info(f"Motion result: {motion_result}")

            action_result = MoveToPose.Result()
            if partial_result:
                if motion_result is None:
                    action_result.result.val = MoveItErrorCodes.FAILURE
                else:
                    action_result.result.val = motion_result.val
            else:
                action_result.result.val = MoveItErrorCodes.FAILURE
        else:
            ik_req = GetPositionIK.Request()
            ik_req.ik_request.group_name = self.move_group_name
            ik_req.ik_request.pose_stamped = goal_pose
            ik_req.ik_request.avoid_collisions = True
            self.moveit2.wait_new_joint_state()
            if self.moveit2.joint_state is not None:
                ik_req.ik_request.robot_state.joint_state = self.moveit2.joint_state
            else:
                self.get_logger().error("Joint state not yet available, cannot compute IK.")
                goal_handle.abort()
                result = MoveToPose.Result()
                result.result.val = MoveItErrorCodes.INVALID_ROBOT_STATE
                return result

            future = self.compute_ik_client.call_async(ik_req)
            rate = self.create_rate(10)
            timpout = 3.0
            start_time = self.get_clock().now()
            while not future.done() and (self.get_clock().now() - start_time).nanoseconds / 1e9 < timpout:
                self.get_logger().info("Waiting for IK solution...")
                rate.sleep()

            if not future.done() or future.result().error_code.val != MoveItErrorCodes.SUCCESS:
                self.get_logger().warn("IK solution not found or call failed.")
                goal_handle.abort()
                result = MoveToPose.Result()
                result.result.val = MoveItErrorCodes.NO_IK_SOLUTION
                return result

            joint_state = future.result().solution.joint_state
            self.get_logger().info(f"IK joint solution: {joint_state.position}")
            joint_name_to_position = dict(zip(joint_state.name, joint_state.position))
            filtered_positions = [joint_name_to_position[name] for name in self.joint_names if name in joint_name_to_position]

            self.moveit2.move_to_configuration(filtered_positions, self.joint_names)
            partial_result = self.moveit2.wait_until_executed()
            motion_result = self.moveit2.get_last_execution_error_code()
            self.get_logger().info(f"Partial result: {partial_result}")
            self.get_logger().info(f"Motion result: {motion_result}")

            action_result = MoveToPose.Result()
            if partial_result:
                if motion_result is None:
                    action_result.result.val = MoveItErrorCodes.FAILURE
                else:
                    action_result.result.val = motion_result.val
            else:
                action_result.result.val = MoveItErrorCodes.FAILURE

        
        goal_handle.succeed()
        return action_result

    def move_to_pose_cancel_callback(self, goal_handle):
        pass

    def move_to_joint_cancel_callback(self, goal_handle):
        pass

    def move_to_joint_callback(self, goal_handle: ServerGoalHandle) -> MoveToJoint.Result:
        
        joints_goal = goal_handle.request.joint_target
        self.get_logger().info(f"Moving to joint: {joints_goal}")

        self.moveit2.move_to_configuration(joints_goal)
        partial_result = self.moveit2.wait_until_executed()
        motion_result = self.moveit2.get_last_execution_error_code()
        self.get_logger().info(f"Partial result: {partial_result}")
        self.get_logger().info(f"Motion result: {motion_result}")

        action_result = MoveToJoint.Result()
        if partial_result:
            if motion_result is None:
                action_result.result.val = MoveItErrorCodes.FAILURE
            else:
                action_result.result.val = motion_result.val
        else:
            action_result.result.val = MoveItErrorCodes.FAILURE
        
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
        self.moveit2.attach_collision_object(id=request.object_id,
                                             link_name=request.target_frame_id,
                                             touch_links=[request.target_frame_id])
        attached = False
        if self.moveit2.update_planning_scene():
            for attached_obj in self.moveit2.planning_scene.robot_state.attached_collision_objects:
                if attached_obj.object.id == request.object_id:
                    attached = True

        if attached:
            self.get_logger().info(f"Object '{request.object_id}' successfully attached.")
            response.success = True
        else:
            self.get_logger().warn(f"Object '{request.object_id}' NOT found in attached objects.")
            response.success = False
                
        return response


    def detach_object_callback(self, request, response):
        self.get_logger().info(f"Detaching object '{request.object_id}'")
        self.moveit2.detach_collision_object(request.object_id)

        detached = False
        if self.moveit2.update_planning_scene():
            still_attached = any(
                attached_obj.object.id == request.object_id
                for attached_obj in self.moveit2.planning_scene.robot_state.attached_collision_objects
            )
            if not still_attached:
                detached = True

        if detached:
            self.get_logger().info(f"Object '{request.object_id}' successfully detached.")
            response.success = True
        else:
            self.get_logger().warn(f"Object '{request.object_id}' still appears attached!")
            response.success = False
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
