cmake_minimum_required(VERSION 3.8)
project(planner_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Release Flags
add_compile_options(-O3 -march=native -DNDEBUG)
# add_compile_options(-march=native -g3 -ggdb3 -O0 -fno-inline -fno-omit-frame-pointer)
# add_compile_options(-fsanitize=address)
# add_link_options(-fsanitize=address)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uav_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(kalman_filter REQUIRED)
find_package(matplotlibcpp REQUIRED)
find_package(rrt_search REQUIRED)
find_package(visibility_graph REQUIRED)

find_package(TBB REQUIRED)
find_package(OpenMP REQUIRED)

# Handle lincov_interface build
add_executable(lincov_interface src/lincov_interface.cpp)
target_include_directories(lincov_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lincov_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  lincov_interface
  matplotlibcpp
  rclcpp
  rclpy
  visualization_msgs
  geometry_msgs
  diagnostic_msgs
  uav_interfaces
  std_srvs
  kalman_filter
  rrt_search
)

target_link_libraries(lincov_interface
  OpenMP::OpenMP_CXX
  TBB::tbb
)

# Handle pdvg_interface build
add_executable(pdvg_interface src/pdvg_interface.cpp)
target_include_directories(pdvg_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pdvg_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  pdvg_interface
  matplotlibcpp
  rclcpp
  rclpy
  uav_interfaces
  std_srvs
  kalman_filter
  visualization_msgs
  diagnostic_msgs
  visibility_graph
  rrt_search
)

target_link_libraries(pdvg_interface
  OpenMP::OpenMP_CXX
  TBB::tbb
)

# Handle state listener build
add_executable(state_listener_csv_node src/state_listener_csv_node.cpp)
target_include_directories(state_listener_csv_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(state_listener_csv_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  state_listener_csv_node
  rclcpp
  std_srvs
  uav_interfaces
)

# Handle sim manager build
add_executable(sim_manager_node src/sim_manager_node.cpp)
target_include_directories(sim_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sim_manager_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  sim_manager_node
  rclcpp
  diagnostic_msgs
  std_srvs
  std_msgs
  uav_interfaces
)

# Install all executables
install(TARGETS
  pdvg_interface
  lincov_interface
  state_listener_csv_node
  sim_manager_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/csv_plotter_run.py
  scripts/mc_plotter_run.py
  scripts/mc_exporter_run.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
