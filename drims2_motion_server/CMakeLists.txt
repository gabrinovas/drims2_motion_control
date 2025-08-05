
cmake_minimum_required(VERSION 3.8)
project(drims2_motion_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)

add_executable(trajectory_loader_server src/trajectory_loader_server.cpp)
ament_target_dependencies(trajectory_loader_server
  rclcpp
  rclcpp_components
  rosidl_default_generators
  rclcpp_action
  std_msgs
  trajectory_msgs
  moveit_msgs
  sensor_msgs
  moveit_ros_planning_interface
  control_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(trajectory_loader_server "${cpp_typesupport_target}" cnr_param::cnr_param)

add_executable(move_to_server src/move_to_server.cpp)
ament_target_dependencies(move_to_server
  rclcpp
  rclcpp_components
  rosidl_default_generators
  rclcpp_action
  std_msgs
  trajectory_msgs
  moveit_msgs
  sensor_msgs
  moveit_ros_planning_interface
  ik_solver_msgs
  geometry_msgs
  control_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(move_to_server "${cpp_typesupport_target}" cnr_param::cnr_param)

add_executable(move_to_conf_server src/move_to_conf_server.cpp)
ament_target_dependencies(move_to_conf_server
  rclcpp
  rclcpp_components
  rosidl_default_generators
  rclcpp_action
  std_msgs
  trajectory_msgs
  moveit_msgs
  sensor_msgs
  moveit_ros_planning_interface
  geometry_msgs
  control_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(move_to_conf_server "${cpp_typesupport_target}" cnr_param::cnr_param)

add_executable(trajectory_loader_test src/trajectory_loader_test.cpp)
ament_target_dependencies(trajectory_loader_test
    rclcpp
    rclcpp_components
    rosidl_default_generators
    rclcpp_action
    std_msgs
    trajectory_msgs
    moveit_msgs
    sensor_msgs
    moveit_ros_planning_interface
    control_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(trajectory_loader_test "${cpp_typesupport_target}" cnr_param::cnr_param)

add_executable(move_to_test src/move_to_test.cpp)
ament_target_dependencies(move_to_test
    rclcpp
    rclcpp_components
    rosidl_default_generators
    rclcpp_action
    std_msgs
    trajectory_msgs
    moveit_msgs
    sensor_msgs
    moveit_ros_planning_interface
    tf2_ros
    control_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(move_to_test "${cpp_typesupport_target}" cnr_param::cnr_param)

add_executable(test_move_group src/test_move_group.cpp)
ament_target_dependencies(test_move_group
    rclcpp
    rclcpp_components
    rosidl_default_generators
    rclcpp_action
    std_msgs
    trajectory_msgs
    moveit_msgs
    sensor_msgs
    moveit_ros_planning_interface
    tf2_ros
    control_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(test_move_group "${cpp_typesupport_target}" cnr_param::cnr_param)

install(TARGETS
  trajectory_loader_server
  move_to_server
  move_to_conf_server
  trajectory_loader_test
  move_to_test
  test_move_group
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME} )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
