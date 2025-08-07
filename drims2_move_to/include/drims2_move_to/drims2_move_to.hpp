#pragma once

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std::chrono_literals;

class MoveTo
{

private:
  std::string group_name_;
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit MoveTo(const std::string& group_name): group_name_(group_name)
  {
    node_ = rclcpp::Node::make_shared("drims2_move_to");
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() {
        executor_->spin();
      });

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name_);
  }

  moveit::core::MoveItErrorCode move_to_joint(const Eigen::VectorXd& q, const std::vector<std::string>& joints_names)
  {
    RCLCPP_INFO(node_->get_logger(),"Starting state monitor");
    if (!move_group_->startStateMonitor())
      {
        RCLCPP_ERROR(node_->get_logger(),"Unable to read current state");
        return false;
      }

    RCLCPP_INFO(node_->get_logger(),"Waiting for robot current state");
    moveit::core::RobotStatePtr robot_current_state = move_group_->getCurrentState();
    std::vector<double> current_configuration;
    for(const std::string& j:joints_names)
      {
        double d = *robot_current_state->getJointPositions(j);
        current_configuration.push_back(d);
      }

    std::string txt_current_state = "Current configuration read:";
    std::vector<std::string> all_joints = move_group_->getActiveJoints();
    for (const auto& j : all_joints)
      {
        const double* positions = robot_current_state->getJointPositions(j);
        if (positions) {
            txt_current_state += "\n\t- Joint: " + j + " -> " + std::to_string(positions[0]);
          } else {
            txt_current_state += "\n\t- Joint: " + j + " -> [no data]";
          }
      }

    RCLCPP_INFO_STREAM(node_->get_logger(),txt_current_state);

    std::map<std::string, double> goal_map;
    for(size_t j=0;j<all_joints.size();j++)
      {
        std::pair<std::string, double> p;
        p.first = all_joints[j];

        auto it = std::find(joints_names.begin(),joints_names.end(),all_joints[j]);
        if(it != joints_names.end())
          p.second = q[std::distance(joints_names.begin(),it)];
        else
          p.second = *robot_current_state.get()->getJointPositions(p.first); //if the joint is not considered, keep it at its current position

        goal_map.insert(p);
      }

    move_group_->setJointValueTarget(goal_map);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    robot_trajectory::RobotTrajectory trajectory(move_group_->getRobotModel(), group_name_);
    moveit::core::MoveItErrorCode res = move_group_->plan(plan);
    if(res != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR(node_->get_logger(),"Planning to location failed!");
        return res;
      }

    res = move_group_->execute(plan);
    if(res != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR(node_->get_logger(),"Execution of trajectory to location failed!");
        return res;
      }

    return res;
  }

  moveit::core::MoveItErrorCode move_to_pose(const geometry_msgs::msg::PoseStamped& pose,
                                             const std::vector<std::string>& joints_names,
                                             const bool cartesian,
                                             const double& cartesian_max_step,
                                             const double& cartesian_fraction_threshold)
  {
    RCLCPP_INFO(node_->get_logger(),"Starting state monitor");
    if (!move_group_->startStateMonitor())
      {
        RCLCPP_ERROR(node_->get_logger(),"Unable to read current state");
        return false;
      }

    RCLCPP_INFO(node_->get_logger(),"Waiting for robot current state");
    moveit::core::RobotStatePtr robot_current_state = move_group_->getCurrentState();
    std::vector<double> current_configuration;
    for(const std::string& j:joints_names)
      {
        double d = *robot_current_state->getJointPositions(j);
        current_configuration.push_back(d);
      }

    std::string txt_current_state = "Current configuration read:";
    std::vector<std::string> all_joints = move_group_->getActiveJoints();
    for (const auto& j : all_joints)
      {
        const double* positions = robot_current_state->getJointPositions(j);
        if (positions) {
            txt_current_state += "\n\t- Joint: " + j + " -> " + std::to_string(positions[0]);
          } else {
            txt_current_state += "\n\t- Joint: " + j + " -> [no data]";
          }
      }

    RCLCPP_INFO_STREAM(node_->get_logger(),txt_current_state);

    move_group_->setPoseTarget(pose);

    moveit::core::MoveItErrorCode res;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if(cartesian)
      {
        RCLCPP_INFO(node_->get_logger(), "Planning Cartesian trajectory...");

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(pose.pose);

        moveit_msgs::msg::RobotTrajectory trajectory_msg;

        double fraction = move_group_->computeCartesianPath(
              waypoints, cartesian_max_step, cartesian_fraction_threshold, trajectory_msg);

        if(fraction < 0.99)
          {
            RCLCPP_WARN(node_->get_logger(), "Cartesian path planning only achieved %.2f%% of the path", fraction * 100.0);
            return moveit::core::MoveItErrorCode::FAILURE;
          }

        plan.trajectory_ = trajectory_msg;

        res = move_group_->execute(plan);
      }
    else
      {
        RCLCPP_INFO(node_->get_logger(), "Planning motion to pose target...");

        res = move_group_->plan(plan);
        if (res == moveit::core::MoveItErrorCode::SUCCESS)
          {
            res = move_group_->execute(plan);
          }
      }

    if (res != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR(node_->get_logger(), "Motion to pose failed!");
      }

    return res;
  }
};
