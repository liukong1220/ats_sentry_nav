cmake_minimum_required(VERSION 3.10)
project(ats_cmd_vel_arbiter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ats_navigation_interfaces REQUIRED)

add_executable(cmd_vel_arbiter_node src/cmd_vel_arbiter_node.cpp)
target_include_directories(cmd_vel_arbiter_node PUBLIC include)
ament_target_dependencies(cmd_vel_arbiter_node
  rclcpp
  geometry_msgs
  std_msgs
  ats_navigation_interfaces
)

install(TARGETS cmd_vel_arbiter_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cmd_vel_arbiter test/test_cmd_vel_arbiter.cpp)
  target_include_directories(test_cmd_vel_arbiter PRIVATE include)
endif()

ament_package()
