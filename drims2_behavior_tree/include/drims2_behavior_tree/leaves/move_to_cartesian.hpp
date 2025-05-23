#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

#include <drims2_msgs/action/move_to_cartesian.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class MoveToCartesian: public BT::RosActionNode<drims2_msgs::action::MoveToCartesian>
{
public:
  MoveToCartesian(const std::string& name,
                  const BT::NodeConfig& conf,
                  const BT::RosNodeParams& params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("pose_target")
      }
    );
  }

  bool setGoal(Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
  BT::NodeStatus onFeedback(const std::shared_ptr<const drims2_msgs::action::MoveToCartesian::Feedback> feedback) override;

// private:
//   std::string ns_, world_;
};

