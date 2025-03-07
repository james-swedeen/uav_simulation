cmake_minimum_required(VERSION 3.8)
project(rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(std_srvs REQUIRED)
find_package(uav_interfaces REQUIRED)

set(rviz_plugins_headers_to_moc
  include/rviz_plugins/lincov_analysis_panel_node.hpp
  include/rviz_plugins/lincov_analysis_panel.hpp
  include/rviz_plugins/pdvg_panel.hpp
  include/rviz_plugins/pdvg_panel_node.hpp
  include/rviz_plugins/gui_tools/q_dropdown_checkbox.hpp
)

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/lincov_analysis_panel_node.cpp
  src/lincov_analysis_panel.cpp
  src/pdvg_panel.cpp
  src/pdvg_panel_node.cpp
  src/q_dropdown_checkbox.cpp
  ${rviz_plugins_headers_to_moc}
)

set(dependencies
  pluginlib
  Qt5
  rclcpp
  rviz_common
  rviz_default_plugins
  uav_interfaces
  std_srvs
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_include_directories(${library_name} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(${library_name}
  rviz_common::rviz_common
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# TODO: Make this specific to this project (not rviz default plugins)
target_compile_definitions(${library_name}
  PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY"
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include/
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

ament_export_include_directories(include)
ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Qt5
  rviz_common
  rclcpp
  uav_interfaces
  std_srvs
)


ament_package()
