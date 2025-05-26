#include <drims2_behavior_tree/leaves/move_to_joint.hpp>

MoveToJoint::MoveToJoint(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: RosActionNode<drims2_msgs::action::MoveToJoint>(name, conf, params)
{
}

bool MoveToJoint::setGoal(RosActionNode::Goal & goal)
{
  RCLCPP_INFO(node_.lock()->get_logger(), "MoveToJoint ticked.");

  auto joint_target = getInput<std::vector<double>>("joint_target");

  if (joint_target) {
    goal.joint_target = joint_target.value();
    return true;
  }
  return false;
}

BT::NodeStatus MoveToJoint::onResultReceived(const RosActionNode::WrappedResult & wr)
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

BT::NodeStatus MoveToJoint::onFailure(BT::ActionNodeErrorCode error)
{
  // RCLCPP_ERROR( node_.lock()->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveToJoint::onFeedback(
  const std::shared_ptr<const drims2_msgs::action::MoveToJoint::Feedback> feedback)
{
  (void) feedback;
  return BT::NodeStatus::RUNNING;
}

// Plugin registration.
// The class MoveToJoint will self register with name  "MoveToJoint".
CreateRosNodePlugin(MoveToJoint, "MoveToJoint");
