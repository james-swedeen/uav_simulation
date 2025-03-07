cmake_minimum_required(VERSION 3.8)
project(sim_kalman_filter)

## Normal Flags
add_compile_options(-std=c++17 -fopenmp)
## Debugging Flags
#add_compile_options(-march=native -g3 -ggdb3 -O0 -fno-inline -fno-omit-frame-pointer)
#add_compile_options(-fsanitize=thread)
#add_link_options(-fsanitize=thread)
# add_compile_options(-fsanitize=address)
# add_link_options(-fsanitize=address)
## Release Flags
add_compile_options(-O3 -march=native -DNDEBUG -fomit-frame-pointer)

##################
## Dependencies ##
##################

find_package(ament_cmake REQUIRED)

find_package(kalman_filter REQUIRED)
find_package(uav_interfaces REQUIRED)

find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

###########
## Build ##
###########

## Diagnostic Wrapper
add_library(diagnostic_wrapper_lib STATIC
  src/diagnostics_wrapper.cpp
)
target_include_directories(diagnostic_wrapper_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(diagnostic_wrapper_lib
  diagnostic_msgs
  rclcpp
)

## Kalman Filter Node
add_executable(kalman_filter_node
  src/kalman_filter_node.cpp
)
target_include_directories(kalman_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(kalman_filter_node
  kalman_filter
  uav_interfaces
  std_srvs
  diagnostic_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  sensor_msgs
  OpenMP
  Eigen3
)
target_link_libraries(kalman_filter_node
  diagnostic_wrapper_lib
  OpenMP::OpenMP_CXX
)

## Feature Sensor Node
add_executable(feature_sensor_node
  src/feature_sensor_main.cpp
  src/feature_sensor_node.cpp
)
target_include_directories(feature_sensor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(feature_sensor_node
  kalman_filter
  uav_interfaces
  rclcpp
  tf2
  tf2_geometry_msgs
  OpenMP
  Eigen3
)
target_link_libraries(kalman_filter_node
  OpenMP::OpenMP_CXX
)

############
## Export ##
############

ament_export_targets(diagnostic_wrapperTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(
  kalman_filter
  uav_interfaces
  std_srvs
  diagnostic_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  Eigen3
  OpenMP
)

#############
## Install ##
#############

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS diagnostic_wrapper_lib
  EXPORT diagnostic_wrapperTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  kalman_filter_node
  feature_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

#############
## Testing ##
#############

ament_package()

