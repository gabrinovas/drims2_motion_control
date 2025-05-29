#include <drims2_behavior_tree/leaves/dice_identification.hpp>

bool DiceIdentification::setRequest(Request::SharedPtr& request)
{
  return true;
}

BT::NodeStatus DiceIdentification::onResponseReceived(const Response::SharedPtr& response)
{
  std::cout << "onResponseReceived " << std::endl;
  if(response->success)
  {
    RCLCPP_INFO(logger(), "SetBool service succeeded.");
    // Set output ports
    setOutput("face_number", response->face_number);
    setOutput("pose", response->pose);
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_INFO(logger(), "SetBool service failed");
    return BT::NodeStatus::FAILURE;
  }
}


// Plugin registration.
// The class DiceIdentification will self register with name  "DiceIdentification".
CreateRosNodePlugin(DiceIdentification, "DiceIdentification");
