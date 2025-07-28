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
class MotionServer(Node):

    def __init__(self):
        super().__init__('motion_server_node')

        # self.declare_parameter('motion_server_config_path', '')
        self.declare_parameter('move_group_name', 'manipulator')
        self.declare_parameter('joint_names', [''])
        self.declare_parameter('base_link_name', 'base_link')
        self.declare_parameter('end_effector_name', 'tool0')
        self.declare_parameter('planner_id', 'RRTConnectkConfigDefault')

        self.declare_parameter(CartesianPlannerParams.PIPELINE, 'pilz_industrial_motion_planner')
        self.declare_parameter(CartesianPlannerParams.PLANNER_ID, 'LIN')
        self.declare_parameter(CartesianPlannerParams.PLANNING_TIME, 50.0)
        self.declare_parameter(CartesianPlannerParams.MAX_VEL_SCALING, 0.05)
        self.declare_parameter(CartesianPlannerParams.MAX_ACC_SCALING, 0.05)

        # self.get_logger().info(self.get_parameter('motion_server_config_path').get_parameter_value().string_value)
        self.move_group_name = self.get_parameter('move_group_name').get_parameter_value().string_value
        self.end_effector_name = self.get_parameter('end_effector_name').get_parameter_value().string_value
        self.base_link_name = self.get_parameter('base_link_name').get_parameter_value().string_value
        self.joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        self.get_logger().info(f"Move group name: {self.move_group_name}")
        self.get_logger().info(f"Joint names: {self.joint_names}")
        self.target_frame = self.get_parameter('base_link_name').get_parameter_value().string_value
        self.get_logger().info(f"Target frame: {self.target_frame}")
        
        self.internal_node = Node(
            'fdsgsdgsedds', use_global_arguments=False, )
        
        # self.internal_executor = rclpy.executors.MultiThreadedExecutor(2)
        # self.internal_executor.add_node(self.internal_node)
        self.internal_node.declare_parameter('synchronous', False)

        callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        try:     
            self.moveit2 = MoveIt2(
                node=self,
                joint_names=self.joint_names,
                base_link_name=self.base_link_name,
                end_effector_name=self.end_effector_name,
                group_name=self.move_group_name,
                callback_group=callback_group,
            )
            self.moveit2.planner_id = self.get_parameter('planner_id').get_parameter_value().string_value

            # Avvia il thread DOPO aver creato tutti i callback

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
        self.move_to_pose_action_server = ActionServer(
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

    def spin_internal_executor(self):
        self.get_logger().info("[InternalExecutor] Spinning internal executor...")
        while not rclpy.ok():
            try:
                self.internal_executor.spin_once(timeout_sec=1.0)
                # rclpy.spin_once(self.internal_node, timeout_sec=1.0)
            except KeyboardInterrupt:
                self.get_logger().info("[InternalExecutor] Shutting down internal executor")
                break
            except Exception as e:
                self.get_logger().error(f"[InternalExecutor] Spin failed: {e}")
        # try:
        #     self.internal_executor.spin()
        # except Exception as e:
        #     self.get_logger().error(f"[InternalExecutor] Spin failed: {e}")
        # self.get_logger().info("[InternalExecutor] Executor stopped.")

    def move_to_pose_callback(self, goal_handle: ServerGoalHandle) -> MoveToPose.Result:
        # robot_model = self.moveit_core.get_robot_model()
        # robot_state = RobotState(robot_model)
        self.robot_arm.set_start_state_to_current_state()
        goal_pose: PoseStamped = goal_handle.request.pose_target
        cartesian_motion = goal_handle.request.cartesian_motion


        self.robot_arm.set_goal_state(pose_stamped_msg = goal_pose, 
                                      pose_link=self.target_frame)
        self.broadcast_pose_goal_tf(goal_pose)

        if cartesian_motion:
            self.get_logger().info(f"Move to Pose with linear cartesian motion.")
            plan_request_parameters = CartesianPlannerParams.get_plan_request_parameters(self,self.moveit_core)
            plan_solution = self.robot_arm.plan(plan_request_parameters)
            self.get_logger().info(f"Plan solution")
        else:
            self.get_logger().info(f"Move to Pose with joint motion.")
            plan_solution = self.robot_arm.plan()
            self.get_logger().info(f"Plan solution")


        action_result = MoveToPose.Result()
        if plan_solution:
            robot_trajectory = plan_solution.trajectory
            self.get_logger().info(f"Trj pre executed")
            execution_result = self.moveit_core.execute(self.move_group_name, robot_trajectory, blocking=True)
            self.get_logger().info(f"Trj executed")

            if execution_result:     
                action_result.result.val = action_result.result.SUCCESS
            else:
                action_result.result.val = action_result.result.FAILURE

        else:
            action_result.result = plan_solution.error_code
            self.get_logger().error(f"Planning to {goal_pose} failed")
        goal_handle.succeed()

        # action_result.result = execution_result.status
        # self.get_logger().info(f'Result of the execution: {action_result.result}')
        # moveit_py.controller_manager.ExecutionStatus

        return action_result

    def move_to_pose_cancel_callback(self, goal_handle):
        pass

    def move_to_joint_cancel_callback(self, goal_handle):
        pass

    def move_to_joint_callback(self, goal_handle: ServerGoalHandle) -> MoveToJoint.Result:
        
        joints_goal = goal_handle.request.joint_target
        self.get_logger().info(f"Moving to joint sfsafdas: {joints_goal}")


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
        planning_scene_monitor = self.moveit_core.get_planning_scene_monitor()
        with planning_scene_monitor.read_write() as scene:
            attached_object = AttachedCollisionObject()
            attached_object.link_name = request.target_frame_id
            attached_object.object.id = request.object_id
            attached_object.object.operation = CollisionObject.ADD
            attached_object.touch_links = [request.target_frame_id]
            
            result = scene.process_attached_collision_object(attached_object)
            response.success = result
            scene.current_state.update()
        
        return response


    def detach_object_callback(self, request, response):
        self.get_logger().info(f"Detaching object '{request.object_id}'")
        planning_scene_monitor = self.moveit_core.get_planning_scene_monitor()
        with planning_scene_monitor.read_write() as scene:
            attached_object = AttachedCollisionObject()
            attached_object.object.id = request.object_id
            attached_object.object.operation = CollisionObject.REMOVE
            
            result = scene.process_attached_collision_object(attached_object)
            response.success = result
            scene.current_state.update()
        
        return response


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
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