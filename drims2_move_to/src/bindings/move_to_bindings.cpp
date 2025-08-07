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
}
