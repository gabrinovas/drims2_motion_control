#include <drims2_behavior_tree/leaves/move_to_pose.hpp>

MoveToPose::MoveToPose(const std::string& name,
                         const BT::NodeConfig& conf,
                         const BT::RosNodeParams& params)
  : RosActionNode<drims2_msgs::action::MoveToPose>(name, conf, params)
{
}

bool MoveToPose::setGoal(RosActionNode::Goal &goal)
{
  auto pose_target = getInput<geometry_msgs::msg::PoseStamped>("pose_target").value();
  goal.pose_target = pose_target;

  // Get _weights;

  return true;
}

BT::NodeStatus MoveToPose::onResultReceived(const RosActionNode::WrappedResult &wr)
{
  return BT::NodeStatus::SUCCESS;
  // RCLCPP_INFO(node_.lock()->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(),
  //             wr.result->ok ? "true" : "false");
  // if (wr.result->ok)
  //   return BT::NodeStatus::SUCCESS;
  // else
  // {
  //   RCLCPP_ERROR_STREAM(node_.lock()->get_logger(), "Error: " << wr.result->error);
  //   return BT::NodeStatus::FAILURE;
  // }
}

BT::NodeStatus MoveToPose::onFailure(BT::ActionNodeErrorCode error)
{
  // RCLCPP_ERROR( node_.lock()->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveToPose::onFeedback(const std::shared_ptr<const drims2_msgs::action::MoveToPose::Feedback> feedback)
{
  (void) feedback;
  return BT::NodeStatus::RUNNING;
}

// Plugin registration.
// The class MoveToPose will self register with name  "MoveToPose".
CreateRosNodePlugin(MoveToPose, "MoveToPose");