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
  src/trajectory/minco_trajectory_optimizer.cpp
  src/trajectory/path_geometry_preprocessor.cpp
  src/trajectory/terminal_yaw_relocation.cpp
  src/trajectory/trajectory_quality_evaluator.cpp
  src/trajectory/yaw_spline_planner.cpp
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

  ament_add_gtest(test_grid_search_clearance test/test_grid_search_clearance.cpp)
  target_link_libraries(test_grid_search_clearance ${PROJECT_NAME})

  ament_add_gtest(
    test_minco_trajectory_optimizer
    test/test_minco_trajectory_optimizer.cpp
  )
  target_link_libraries(test_minco_trajectory_optimizer ${PROJECT_NAME})

  ament_add_gtest(test_yaw_spline_planner test/test_yaw_spline_planner.cpp)
  target_link_libraries(test_yaw_spline_planner ${PROJECT_NAME})

  ament_add_gtest(test_yaw_authority_policy test/test_yaw_authority_policy.cpp)
  target_link_libraries(test_yaw_authority_policy ${PROJECT_NAME})

  ament_add_gtest(test_planning_map_snapshot test/test_planning_map_snapshot.cpp)
  target_link_libraries(test_planning_map_snapshot ${PROJECT_NAME})

  ament_add_gtest(test_atomic_planning_contract test/test_atomic_planning_contract.cpp)
  target_link_libraries(test_atomic_planning_contract ${PROJECT_NAME})

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
