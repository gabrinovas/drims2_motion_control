#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

#include <drims2_msgs/action/move_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace BT
{

template <>
inline std::vector<double> convertFromString(StringView str)
{
    std::vector<double> vec;
    auto parts = splitString(str, ';');
    for(auto & element: parts)
    {
      vec.push_back(convertFromString<double>(element));
      std::cerr << element << std::endl;
    }
//     std::string s(str);
//     std::istringstream iss(s);
//     char c;
//     double value;

//     // Skip any whitespace and check for optional opening bracket '['
//     while (iss >> std::ws && iss.peek() == '[')
//     {
//         iss.get(); // Consume the opening bracket
//         break;
//     }

//     while (true)
//     {
//         iss >> std::ws; // Skip whitespace
//         if (!(iss >> value))
//         {
//             throw BT::RuntimeError("Failed to parse vector<double>: expected a number");
//         }

//         vec.push_back(value);

//         iss >> std::ws; // Skip whitespace
//         if (iss.peek() == ',')
//         {
//             iss.get(); // Consume comma and continue
//         }
//         else if (iss.peek() == ']')
//         {
//             iss.get(); // Consume closing bracket and stop
//             break;
//         }
//         else if (iss.eof())
//         {
//             break; // End of string, no closing bracket
//         }
//         else
//         {
//             throw BT::RuntimeError("Failed to parse vector<double>: expected ',' or ']'");
//         }
//     }

    return vec;
}

}  // namespace BT

class MoveToPose: public BT::RosActionNode<drims2_msgs::action::MoveToPose>
{
public:
MoveToPose(const std::string& name,
                  const BT::NodeConfig& conf,
                  const BT::RosNodeParams& params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("pose_target"),
        BT::InputPort<std::string>("frame_id"),
        BT::InputPort<std::vector<double>>("position"),
        BT::InputPort<std::vector<double>>("orientation"),
        
      }
    );
  }

  bool setGoal(Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
  BT::NodeStatus onFeedback(const std::shared_ptr<const drims2_msgs::action::MoveToPose::Feedback> feedback) override;

// private:
//   std::string ns_, world_;
};


