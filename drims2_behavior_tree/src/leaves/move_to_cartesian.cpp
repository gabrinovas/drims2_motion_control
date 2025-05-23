#include <drims2_behavior_tree/leaves/move_to_cartesian.hpp>

MoveToCartesian::MoveToCartesian(const std::string& name,
                         const BT::NodeConfig& conf,
                         const BT::RosNodeParams& params)
  : RosActionNode<drims2_msgs::action::MoveToCartesian>(name, conf, params)
{
}

bool MoveToCartesian::setGoal(RosActionNode::Goal &goal)
{
  auto pose_target = getInput<geometry_msgs::msg::PoseStamped>("pose_target").value();
  goal.pose_target = pose_target;

  // Get _weights;

  return true;
}

BT::NodeStatus MoveToCartesian::onResultReceived(const RosActionNode::WrappedResult &wr)
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

BT::NodeStatus MoveToCartesian::onFailure(BT::ActionNodeErrorCode error)
{
  // RCLCPP_ERROR( node_.lock()->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveToCartesian::onFeedback(const std::shared_ptr<const drims2_msgs::action::MoveToCartesian::Feedback> feedback)
{
  (void) feedback;
  return BT::NodeStatus::RUNNING;
}

// Plugin registration.
// The class MoveToCartesian will self register with name  "MoveToCartesian".
CreateRosNodePlugin(MoveToCartesian, "MoveToCartesian");