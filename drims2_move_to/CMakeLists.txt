cmake_minimum_required(VERSION 3.16)
project(drims2_move_to VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_BUILD_TYPE "Release")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# ------------------------------------------------------------------------------
# Core C++ Library
# ------------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/drims2_move_to.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  moveit_ros_planning_interface
)

# ------------------------------------------------------------------------------
# Python Bindings
# ------------------------------------------------------------------------------
add_library(${PROJECT_NAME}_py MODULE src/bindings/move_to_bindings.cpp)

target_link_libraries(${PROJECT_NAME}_py PRIVATE
  ${PROJECT_NAME} 
  pybind11::module
)

target_include_directories(${PROJECT_NAME}_py PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${Python3_INCLUDE_DIRS}  
)

set_target_properties(${PROJECT_NAME}_py PROPERTIES
  PREFIX ""
  OUTPUT_NAME "drims2_move_to_py"
  SUFFIX ".so"
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/pybind11
)

add_custom_command(TARGET ${PROJECT_NAME}_py POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy
          $<TARGET_FILE:${PROJECT_NAME}_py>
          ${CMAKE_CURRENT_SOURCE_DIR}/scripts/drims2_move_to/drims2_move_to_py.so
)

# ------------------------------------------------------------------------------
# Install Targets
# ------------------------------------------------------------------------------
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_py
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(DIRECTORY scripts/
#   DESTINATION lib/${PROJECT_NAME}
# )
ament_python_install_package(drims2_move_to
  PACKAGE_DIR scripts/drims2_move_to
)
ament_package()
