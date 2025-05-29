from rclpy.node import Node
from moveit.planning import MoveItPy, PlanRequestParameters


class CartesianPlannerParams:
    PIPELINE = 'cartesian_planner_config.planning_pipeline'
    PLANNER_ID = 'cartesian_planner_config.planner_id'
    PLANNING_TIME = 'cartesian_planner_config.planning_time'
    MAX_VEL_SCALING = 'cartesian_planner_config.max_velocity_scaling_factor'
    MAX_ACC_SCALING = 'cartesian_planner_config.max_acceleration_scaling_factor'

    @staticmethod
    def get_plan_request_parameters(node: Node, moveit_core: MoveItPy) -> PlanRequestParameters:

        plan_request_param = PlanRequestParameters(moveit_core)
        plan_request_param.planning_pipeline = node.get_parameter(CartesianPlannerParams.PIPELINE).get_parameter_value().string_value
        plan_request_param.planner_id = node.get_parameter(CartesianPlannerParams.PLANNER_ID).get_parameter_value().string_value
        plan_request_param.planning_time = node.get_parameter(CartesianPlannerParams.PLANNING_TIME).get_parameter_value().double_value
        plan_request_param.max_velocity_scaling_factor = node.get_parameter(CartesianPlannerParams.MAX_VEL_SCALING).get_parameter_value().double_value
        plan_request_param.max_acceleration_scaling_factor = node.get_parameter(CartesianPlannerParams.MAX_ACC_SCALING).get_parameter_value().double_value

        return plan_request_param
