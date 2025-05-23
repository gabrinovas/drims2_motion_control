import math


from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)

import rclpy
from rclpy.logging import get_logger

def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    # plan to goal
    logger = get_logger('plan_and_execute')
    logger.info('Planning trajectory')
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(multi_plan_parameters=multi_plan_parameters)
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(single_plan_parameters=single_plan_parameters)
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info('Executing plan')
        robot_trajectory = plan_result.trajectory
        execute_result = robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error('Planning failed')
        execute_result = False

    time.sleep(sleep_time)
    return execute_result

def main(args=None):
    rclpy.init(args=args)
    logger = get_logger('drims2_motion_server')

    # instantiate MoveItPy instance and get planning component
    drims2_motion_server = MoveItPy(node_name='drims2_motion_server')
    logger.info('MoveItPy instance created')

    arm = drims2_motion_server.get_planning_component('arm')

    # instantiate a RobotState instance using the current robot model
    robot_model = drims2_motion_server.get_robot_model()
    robot_state = RobotState(robot_model)

    arm_plan_request_params = PlanRequestParameters(
        drims2_motion_server,
        'ompl_rrtc',
    )

    arm_plan_request_params.max_acceleration_scaling_factor = 0.5  # Set 0.0 ~ 1.0
    arm_plan_request_params.max_velocity_scaling_factor = 0.5  # Set 0.0 ~ 1.0

    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name='vertical')
    plan_and_execute(
        drims2_motion_server,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    joint_names = [
        'crane_x7_shoulder_fixed_part_pan_joint',
        'crane_x7_shoulder_revolute_part_tilt_joint',
        'crane_x7_upper_arm_revolute_part_twist_joint',
        'crane_x7_upper_arm_revolute_part_rotate_joint',
        'crane_x7_lower_arm_fixed_part_joint',
        'crane_x7_lower_arm_revolute_part_joint',
        'crane_x7_wrist_joint',
        ]
    target_joint_value = math.radians(-45.0)

    # 各関節角度を順番に-45[deg]ずつ動かす
    for joint_name in joint_names:
        arm.set_start_state_to_current_state()

        joint_values = {joint_name: target_joint_value}
        robot_state.joint_positions = joint_values
        joint_constraint = construct_joint_constraint(
            robot_state=robot_state,
            joint_model_group=drims2_motion_server.get_robot_model().get_joint_model_group('arm'),
        )
        arm.set_goal_state(motion_plan_constraints=[joint_constraint])

        plan_and_execute(
            drims2_motion_server,
            arm,
            logger,
            single_plan_parameters=arm_plan_request_params,
        )

    # SRDFに定義されている'vertical'の姿勢にする
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name='vertical')
    plan_and_execute(
        drims2_motion_server,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()