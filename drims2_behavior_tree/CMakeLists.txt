cmake_minimum_required(VERSION 3.8)
project(drims2_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(drims2_msgs REQUIRED)

set(dependencies
  rclcpp
  behaviortree_cpp
  behaviortree_ros2
  drims2_msgs
)
    
include_directories(include)

add_executable(bt_executer_node
    src/bt_executer_node.cpp
)
ament_target_dependencies(bt_executer_node ${dependencies})

add_library(move_to_cartesian SHARED
    src/leaves/move_to_cartesian.cpp
)
list(APPEND plugin_libs move_to_cartesian)

# add_library(move_to_joint SHARED
#     src/leaves/move_to_joint.cpp
# )
# list(APPEND plugin_libs move_to_joint)

foreach(bt_plugin ${plugin_libs})
  # target_link_libraries(${bt_plugin} ${dependencies})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  endforeach()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(TARGETS
  bt_executer_node
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
    DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()