#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "MoveTo.hpp"

namespace py = pybind11;

PYBIND11_MODULE(drims2_bindings, m) {
    py::class_<MoveTo>(m, "MoveTo")
        .def(py::init<const std::string&>(), py::arg("group_name"))
        .def("move_to_joint", &MoveTo::move_to_joint,
             py::arg("q"),
             py::arg("joints_names"),
             R"pbdoc(
                Move to a specified joint configuration.

                Args:
                    q: Eigen vector of joint positions.
                    joints_names: List of joint names corresponding to `q`.

                Returns:
                    MoveItErrorCode indicating success or failure.
             )pbdoc")
        .def("move_to_pose", &MoveTo::move_to_pose,
             py::arg("pose"),
             py::arg("joints_names"),
             py::arg("cartesian"),
             py::arg("cartesian_max_step"),
             py::arg("cartesian_fraction_threshold"),
             R"pbdoc(
                Move to a specified pose.

                Args:
                    pose: PoseStamped target.
                    joints_names: List of joints to control.
                    cartesian: Whether to use Cartesian motion planning.
                    cartesian_max_step: Max step for Cartesian planning.
                    cartesian_fraction_threshold: Required fraction of path to be planned successfully.

                Returns:
                    MoveItErrorCode indicating success or failure.
             )pbdoc");
}
