#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "drims2_move_to/drims2_move_to.hpp"

namespace py = pybind11;

PYBIND11_MODULE(drims2_move_to_py, m) {
    // --- Classe MoveTo ---
    py::class_<MoveTo>(m, "MoveTo")
        .def(py::init([](const std::string& group_name) {
            if (!rclcpp::ok()) {
                int argc = 0;
                char** argv = nullptr;
                rclcpp::init(argc, argv);
            }
            return std::make_unique<MoveTo>(group_name);
        }), py::arg("group_name"))
        .def("move_to_joint", &MoveTo::move_to_joint,
             py::arg("q"),
             py::arg("joints_names"))
        .def("move_to_pose", [](MoveTo& self,
                                py::dict pose_dict,
                                std::vector<std::string> joints_names,
                                bool cartesian,
                                double cartesian_max_step,
                                double cartesian_fraction_threshold) {
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.frame_id = pose_dict["frame_id"].cast<std::string>();
            pose_msg.pose.position.x = pose_dict["position"]["x"].cast<double>();
            pose_msg.pose.position.y = pose_dict["position"]["y"].cast<double>();
            pose_msg.pose.position.z = pose_dict["position"]["z"].cast<double>();
            pose_msg.pose.orientation.x = pose_dict["orientation"]["x"].cast<double>();
            pose_msg.pose.orientation.y = pose_dict["orientation"]["y"].cast<double>();
            pose_msg.pose.orientation.z = pose_dict["orientation"]["z"].cast<double>();
            pose_msg.pose.orientation.w = pose_dict["orientation"]["w"].cast<double>();
            return self.move_to_pose(pose_msg, joints_names, cartesian, cartesian_max_step, cartesian_fraction_threshold);
        }, py::arg("pose"), py::arg("joints_names"),
           py::arg("cartesian"), py::arg("cartesian_max_step"),
           py::arg("cartesian_fraction_threshold"));

    // // --- Classe MoveItErrorCodes ---
    // py::class_<moveit_msgs::msg::MoveItErrorCodes>(m, "MoveItErrorCodes")
    //     .def(py::init<>())
    //     .def_readwrite("val", &moveit_msgs::msg::MoveItErrorCodes::val);

    // // --- Attributi costanti dei codici MoveIt ---
    // m.attr("MOVEIT_SUCCESS") = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    // m.attr("MOVEIT_FAILURE") = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    // m.attr("MOVEIT_PLANNING_FAILED") = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    // m.attr("MOVEIT_INVALID_MOTION_PLAN") = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
    // m.attr("MOVEIT_MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE") =
    //     moveit_msgs::msg::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE;
    // m.attr("MOVEIT_CONTROL_FAILED") = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
    // m.attr("MOVEIT_UNABLE_TO_AQUIRE_SENSOR_DATA") =
    //     moveit_msgs::msg::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA;
    // m.attr("MOVEIT_TIMED_OUT") = moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT;
    // m.attr("MOVEIT_PREEMPTED") = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
    // m.attr("MOVEIT_START_STATE_IN_COLLISION") = moveit_msgs::msg::MoveItErrorCodes::START_STATE_IN_COLLISION;
    // m.attr("MOVEIT_START_STATE_VIOLATES_PATH_CONSTRAINTS") =
    //     moveit_msgs::msg::MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS;
    // m.attr("MOVEIT_START_STATE_INVALID") = moveit_msgs::msg::MoveItErrorCodes::START_STATE_INVALID;
    // m.attr("MOVEIT_GOAL_IN_COLLISION") = moveit_msgs::msg::MoveItErrorCodes::GOAL_IN_COLLISION;
    // m.attr("MOVEIT_GOAL_VIOLATES_PATH_CONSTRAINTS") =
    //     moveit_msgs::msg::MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS;
    // m.attr("MOVEIT_GOAL_CONSTRAINTS_VIOLATED") =
    //     moveit_msgs::msg::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED;
    // m.attr("MOVEIT_GOAL_STATE_INVALID") = moveit_msgs::msg::MoveItErrorCodes::GOAL_STATE_INVALID;
    // m.attr("MOVEIT_UNRECOGNIZED_GOAL_TYPE") = moveit_msgs::msg::MoveItErrorCodes::UNRECOGNIZED_GOAL_TYPE;
    // m.attr("MOVEIT_INVALID_GROUP_NAME") = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
    // m.attr("MOVEIT_INVALID_GOAL_CONSTRAINTS") =
    //     moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    // m.attr("MOVEIT_INVALID_ROBOT_STATE") = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
    // m.attr("MOVEIT_INVALID_LINK_NAME") = moveit_msgs::msg::MoveItErrorCodes::INVALID_LINK_NAME;
    // m.attr("MOVEIT_INVALID_OBJECT_NAME") = moveit_msgs::msg::MoveItErrorCodes::INVALID_OBJECT_NAME;
    // m.attr("MOVEIT_FRAME_TRANSFORM_FAILURE") =
    //     moveit_msgs::msg::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
    // m.attr("MOVEIT_COLLISION_CHECKING_UNAVAILABLE") =
    //     moveit_msgs::msg::MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE;
    // m.attr("MOVEIT_ROBOT_STATE_STALE") = moveit_msgs::msg::MoveItErrorCodes::ROBOT_STATE_STALE;
    // m.attr("MOVEIT_SENSOR_INFO_STALE") = moveit_msgs::msg::MoveItErrorCodes::SENSOR_INFO_STALE;
    // m.attr("MOVEIT_COMMUNICATION_FAILURE") =
    //     moveit_msgs::msg::MoveItErrorCodes::COMMUNICATION_FAILURE;
    // m.attr("MOVEIT_CRASH") = moveit_msgs::msg::MoveItErrorCodes::CRASH;
    // m.attr("MOVEIT_ABORT") = moveit_msgs::msg::MoveItErrorCodes::ABORT;
    // m.attr("MOVEIT_NO_IK_SOLUTION") = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
}
