cmake_minimum_required(VERSION 3.10)
project(minco_planner)

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
  src/nodes/planning_map_snapshot.cpp
  src/planning/grid_astar.cpp
  src/planning/grid_jps.cpp
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_grid_jps test/test_grid_jps.cpp)
  target_link_libraries(test_grid_jps ${PROJECT_NAME})
  ament_add_gtest(test_reference_progress_projection test/test_reference_progress_projection.cpp)
  target_link_libraries(test_reference_progress_projection ${PROJECT_NAME})

  ament_add_gtest(test_grid_search_clearance test/test_grid_search_clearance.cpp)
  target_link_libraries(test_grid_search_clearance ${PROJECT_NAME})

  ament_add_gtest(
    test_minco_trajectory_optimizer
    test/test_minco_trajectory_optimizer.cpp
  )
  target_link_libraries(test_minco_trajectory_optimizer ${PROJECT_NAME})

  ament_add_gtest(test_minco_joint_optimizer test/test_minco_joint_optimizer.cpp)
  target_link_libraries(test_minco_joint_optimizer ${PROJECT_NAME})

  ament_add_gtest(test_planner_debug_visualizer test/test_planner_debug_visualizer.cpp)
  target_link_libraries(test_planner_debug_visualizer ${PROJECT_NAME})

  ament_add_gtest(test_yaw_spline_planner test/test_yaw_spline_planner.cpp)
  target_link_libraries(test_yaw_spline_planner ${PROJECT_NAME})

  ament_add_gtest(test_tunnel_yaw_alignment test/test_tunnel_yaw_alignment.cpp)
  target_link_libraries(test_tunnel_yaw_alignment ${PROJECT_NAME})

  ament_add_gtest(test_wheel_speed_time_scaling test/test_wheel_speed_time_scaling.cpp)
  target_link_libraries(test_wheel_speed_time_scaling ${PROJECT_NAME})

  ament_add_gtest(test_yaw_authority_policy test/test_yaw_authority_policy.cpp)
  target_link_libraries(test_yaw_authority_policy ${PROJECT_NAME})

  ament_add_gtest(test_planning_map_snapshot test/test_planning_map_snapshot.cpp)
  target_link_libraries(test_planning_map_snapshot ${PROJECT_NAME})

  ament_add_gtest(test_atomic_planning_contract test/test_atomic_planning_contract.cpp)
  target_link_libraries(test_atomic_planning_contract ${PROJECT_NAME})

  # Drives the real MincoPlannerNode in-process (same approach as minco_sandbox_cli).
  ament_add_gtest(test_minco_planner_node_ongrid test/test_minco_planner_node_ongrid.cpp)
  target_link_libraries(test_minco_planner_node_ongrid ${PROJECT_NAME})

  ament_add_gtest(test_reference_path_timing test/test_reference_path_timing.cpp)
  target_link_libraries(test_reference_path_timing ${PROJECT_NAME})

  ament_add_gtest(test_footprint_swept_safety test/test_footprint_swept_safety.cpp)
  target_link_libraries(test_footprint_swept_safety ${PROJECT_NAME})

  ament_add_gtest(test_local_collision_repair test/test_local_collision_repair.cpp)
  target_link_libraries(test_local_collision_repair ${PROJECT_NAME})

  ament_add_gtest(test_terminal_yaw_relocation test/test_terminal_yaw_relocation.cpp)
  target_link_libraries(test_terminal_yaw_relocation ${PROJECT_NAME})

  ament_add_gtest(test_goal_pose_admission test/test_goal_pose_admission.cpp)
  target_link_libraries(test_goal_pose_admission ${PROJECT_NAME})

  ament_add_gtest(test_escape_prefix test/test_escape_prefix.cpp)
  target_link_libraries(test_escape_prefix ${PROJECT_NAME})

  ament_add_gtest(test_clearance_ladder test/test_clearance_ladder.cpp)
  target_link_libraries(test_clearance_ladder ${PROJECT_NAME})

  ament_add_gtest(test_graph_search_failure test/test_graph_search_failure.cpp)
  target_link_libraries(test_graph_search_failure ${PROJECT_NAME})

  ament_add_gtest(
    test_path_geometry_preprocessor
    test/test_path_geometry_preprocessor.cpp
  )
  target_link_libraries(test_path_geometry_preprocessor ${PROJECT_NAME})

  ament_add_gtest(test_minco_time_allocator test/test_minco_time_allocator.cpp)
  target_link_libraries(test_minco_time_allocator ${PROJECT_NAME})

  ament_add_gtest(
    test_trajectory_quality_evaluator
    test/test_trajectory_quality_evaluator.cpp
  )
  target_link_libraries(test_trajectory_quality_evaluator ${PROJECT_NAME})
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
