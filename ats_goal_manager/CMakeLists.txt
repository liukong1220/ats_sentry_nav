cmake_minimum_required(VERSION 3.10)
project(ats_goal_manager)

# 未指定 CMAKE_BUILD_TYPE 时 colcon 不传 -O，整包以 -O0 编译（报告 5.1.6 H）；默认取 Release。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ats_navigation_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(ats_goal_manager_node
  src/ats_goal_manager_node.cpp
  src/ats_goal_manager_callbacks.cpp
  src/ats_goal_manager_tick.cpp
  src/ats_goal_manager_helpers.cpp
)
target_include_directories(ats_goal_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ats_goal_manager_node
  ats_navigation_interfaces geometry_msgs nav_msgs rclcpp rclcpp_action std_msgs tf2
  tf2_geometry_msgs tf2_ros
)

install(TARGETS ats_goal_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
