#include <drims2_behavior_tree/leaves/gripper_command.hpp>

GripperCommand::GripperCommand(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: RosActionNode<control_msgs::action::GripperCommand>(name, conf, params)
{
  // FIX: Proper parameter access
  auto node = params.nh;
  if (node->has_parameter("gripper_type")) {
    gripper_type_ = node->get_parameter("gripper_type").as_string();
  } else {
    // Try to get from global parameters
    gripper_type_ = node->get_parameter("gripper_type").as_string();
  }
  
  RCLCPP_INFO(node->get_logger(), "GripperCommand initialized with type: %s", gripper_type_.c_str());
}
bool GripperCommand::setGoal(RosActionNode::Goal & goal)
{
  RCLCPP_INFO(node_.lock()->get_logger(), "GripperCommand ticked.");

  // Required fields: position, max_effort
  double position;
  if (!getInput("position", position)) {
    throw BT::RuntimeError("Missing parameter [position]");
  }
  double max_effort;
  if (!getInput("max_effort", max_effort)) {
    throw BT::RuntimeError("Missing parameter [max_effort]");
  }

  // Handle different gripper types
  if (gripper_type_ == "onrobot_2fg7") {
    // OnRobot 2FG7: Convert total opening to finger position
    // Total opening: 0.035m (closed) to 0.075m (open)
    // Each finger moves 0.0 to 0.020m from center
    double total_opening = std::max(0.035, std::min(0.075, position));
    double finger_position = (total_opening - 0.035) / 2.0;
    goal.command.position = finger_position;
    RCLCPP_INFO(node_.lock()->get_logger(), 
                "OnRobot 2FG7: total opening=%.3fm, finger position=%.3fm", 
                total_opening, finger_position);
  } else {
    // Robotiq: Use position directly (0.0 to 0.79)
    goal.command.position = position;
    RCLCPP_INFO(node_.lock()->get_logger(), 
                "Robotiq: position=%.3fm", position);
  }

  goal.command.max_effort = max_effort;
  return true;
}

BT::NodeStatus GripperCommand::onResultReceived(const RosActionNode::WrappedResult & wr)
{
  RCLCPP_INFO(node_.lock()->get_logger(), "%s: onResultReceived.", name().c_str());
  if(wr.result->reached_goal)
  {
    return  BT::NodeStatus::SUCCESS;
  }
  else if (wr.result->stalled)
  {
    RCLCPP_INFO(node_.lock()->get_logger(), "Gripper stalled");
    return BT::NodeStatus::SUCCESS;
  }
  else{
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus GripperCommand::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_.lock()->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus GripperCommand::onFeedback(
  const std::shared_ptr<const control_msgs::action::GripperCommand::Feedback> feedback)
{
  (void) feedback;
  return BT::NodeStatus::RUNNING;
}

// Plugin registration.
// The class GripperCommand will self register with name  "GripperCommand".
CreateRosNodePlugin(GripperCommand, "GripperCommand");
