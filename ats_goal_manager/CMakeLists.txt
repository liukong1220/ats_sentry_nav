cmake_minimum_required(VERSION 3.10)
project(ats_goal_manager)

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

add_executable(ats_goal_manager_node src/ats_goal_manager_node.cpp)
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

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_goal_lifecycle test/test_goal_lifecycle.cpp)
  target_include_directories(test_goal_lifecycle PRIVATE include)

  ament_add_gtest(test_plan_progress_watchdog test/test_plan_progress_watchdog.cpp)
  target_include_directories(test_plan_progress_watchdog PRIVATE include)

  ament_add_gtest(test_planning_snapshot_safety test/test_planning_snapshot_safety.cpp)
  target_include_directories(test_planning_snapshot_safety PRIVATE include)
  ament_target_dependencies(test_planning_snapshot_safety ats_navigation_interfaces geometry_msgs)

  ament_add_pytest_test(test_goal_manager_epoch test/test_goal_manager_epoch.py
    TIMEOUT 40
  )
endif()

ament_package()
