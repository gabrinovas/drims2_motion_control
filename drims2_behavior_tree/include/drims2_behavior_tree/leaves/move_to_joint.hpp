#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

#include <drims2_msgs/action/move_to_joint.hpp>


class MoveToJoint: public BT::RosActionNode<drims2_msgs::action::MoveToJoint>
{
public:
MoveToJoint(const std::string& name,
                  const BT::NodeConfig& conf,
                  const BT::RosNodeParams& params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::vector<double>>("joint_target"),
      }
    );
  }

  bool setGoal(Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
  BT::NodeStatus onFeedback(const std::shared_ptr<const drims2_msgs::action::MoveToJoint::Feedback> feedback) override;

};


