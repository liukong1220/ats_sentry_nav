cmake_minimum_required(VERSION 3.10)
project(small_gicp_relocalization)

# 未指定 CMAKE_BUILD_TYPE 时 colcon 不传 -O，整包以 -O0 编译（报告 5.1.6 H）；默认取 Release。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

## std::optional is used by the relocalization and fusion nodes.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenMP REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/localization_fusion_node.cpp
  src/localization_fusion_observation.cpp
  src/small_gicp_relocalization.cpp
  src/small_gicp_relocalization_align.cpp
  src/small_gicp_relocalization_state.cpp
)

target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN small_gicp_relocalization::SmallGicpRelocalizationNode
  EXECUTABLE ${PROJECT_NAME}_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN small_gicp_relocalization::LocalizationFusionNode
  EXECUTABLE localization_fusion_node
)

#############
## Testing ##
#############

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
