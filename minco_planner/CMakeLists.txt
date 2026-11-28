cmake_minimum_required(VERSION 3.10)
project(minco_planner)

# 未指定 CMAKE_BUILD_TYPE 时 colcon 不传 -O，整包以 -O0 编译（报告 5.1.6 H）；默认取 Release。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/debug/planner_debug_visualizer.cpp
  src/nodes/minco_planner_node.cpp
  src/nodes/minco_planner_params.cpp
  src/nodes/minco_planner_search.cpp
  src/nodes/minco_planner_route.cpp
  src/nodes/minco_planner_route_mapless.cpp
  src/nodes/minco_planner_runtime.cpp
  src/nodes/minco_planner_plan_goal.cpp
  src/nodes/minco_planner_recovery_commit.cpp
  src/nodes/planning_map_snapshot.cpp
  src/planning/grid_search.cpp
  src/planning/local_route.cpp
  src/safety/footprint_safety_checker.cpp
  src/safety/local_collision_repair.cpp
  src/safety/goal_pose_admission.cpp
  src/trajectory/minco_time_allocator.cpp
  src/trajectory/minco_joint_optimizer.cpp
  src/trajectory/minco_trajectory_optimizer.cpp
  src/trajectory/path_geometry_preprocessor.cpp
  src/trajectory/terminal_yaw_relocation.cpp
  src/trajectory/trajectory_quality_evaluator.cpp
  src/trajectory/yaw_spline_planner.cpp
  src/trajectory/tunnel_yaw_alignment.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN minco_planner::MincoPlannerNode
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md THIRD_PARTY_NOTICES.md
  DESTINATION share/${PROJECT_NAME}
)

# 参考/minco_sandbox 调试平台的适配器: 在本进程里驱动真实 MincoPlannerNode 并导出参考轨迹。
# 它是调试工具, 默认不编译, 常规构建和整仓构建不受影响。需要时:
#   colcon build --base-paths src --packages-select minco_planner \
#     --cmake-args -DMINCO_BUILD_SANDBOX_CLI=ON
option(MINCO_BUILD_SANDBOX_CLI "Build minco_sandbox_cli for the MINCO debug sandbox" OFF)
if(MINCO_BUILD_SANDBOX_CLI)
  add_executable(minco_sandbox_cli tools/minco_sandbox_cli.cpp)
  target_link_libraries(minco_sandbox_cli ${PROJECT_NAME})
  install(TARGETS minco_sandbox_cli DESTINATION lib/${PROJECT_NAME})
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
