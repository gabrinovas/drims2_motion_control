#include <drims2_behavior_tree/leaves/move_to_pose.hpp>

MoveToPose::MoveToPose(const std::string& name,
                         const BT::NodeConfig& conf,
                         const BT::RosNodeParams& params)
  : RosActionNode<drims2_msgs::action::MoveToPose>(name, conf, params)
{
}

bool MoveToPose::setGoal(RosActionNode::Goal &goal)
{
  auto pose_target = getInput<geometry_msgs::msg::PoseStamped>("pose_target");
  std::cerr << "hereee pre" << std::endl;

  std::vector<double> prova = getInput<std::vector<double>>("position").value();
  std::cerr << "hereee" << std::endl;
  for(const auto & element : prova) 
  {
    std::cerr << element << std::endl;
  }
  if (pose_target) {
    goal.pose_target = pose_target.value();
    return true;
  }

  // If pose_target is not set, build it from components
  geometry_msgs::msg::PoseStamped pose;

  // Required fields: frame_id, position (3), orientation (4)
  if (!getInput("frame_id", pose.header.frame_id)) {
    throw BT::RuntimeError("Missing parameter [frame_id]");
  }

  std::vector<double> position;
  if (!getInput("position", position) || position.size() != 3) {
    throw BT::RuntimeError("Invalid or missing parameter [position]. Expected 3 elements");
  }

  std::vector<double> orientation;
  if (!getInput("orientation", orientation) || orientation.size() != 4) {
    throw BT::RuntimeError("Invalid or missing parameter [orientation]. Expected 4 elements");
  }

  pose.header.stamp = node_.lock()->now();  // timestamp current time
  pose.pose.position.x = position[0];
  pose.pose.position.y = position[1];
  pose.pose.position.z = position[2];
  pose.pose.orientation.x = orientation[0];
  pose.pose.orientation.y = orientation[1];
  pose.pose.orientation.z = orientation[2];
  pose.pose.orientation.w = orientation[3];

  goal.pose_target = pose;
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
