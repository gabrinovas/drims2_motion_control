cmake_minimum_required(VERSION 3.8)
project(drims2_move_to)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Include headers
include_directories(include)

# Create the library from the source file
add_library(${PROJECT_NAME}
  src/drims2_move_to.cpp
)

# Link libraries to dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  moveit_ros_planning_interface
)

# Optionally install headers if any
install(DIRECTORY include/
  DESTINATION include/
)

# Install the compiled library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Export include and library for other packages
ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(rclcpp moveit_ros_planning_interface)

# Linting for testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
