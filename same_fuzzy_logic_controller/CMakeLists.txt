cmake_minimum_required(VERSION 3.5)
project(same_fuzzy_logic_controller)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)

nav2_package()

include_directories(
  include
)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  nav_2d_msgs
  dwb_msgs
  nav2_costmap_2d
  pluginlib
  sensor_msgs
  visualization_msgs
  nav_2d_utils
  nav_msgs
  tf2_ros
  nav2_util
  nav2_core
)

add_library(same_fuzzy_logic_controller SHARED
  src/same_fuzzy_logic_controller.cpp
  src/publisher.cpp
  src/illegal_trajectory_tracker.cpp
  src/trajectory_utils.cpp
)

ament_target_dependencies(same_fuzzy_logic_controller
  ${dependencies}
)

install(TARGETS same_fuzzy_logic_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

target_link_directories(same_fuzzy_logic_controller PUBLIC "/home/pgarcia/fuzzylite/fuzzylite/release/bin")#---------
target_link_libraries(same_fuzzy_logic_controller fuzzylite)

#if(BUILD_TESTING)
 # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
 # set(ament_cmake_copyright_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()

 # find_package(ament_cmake_gtest REQUIRED)
#  add_subdirectory(test)
#endif()

ament_export_include_directories(include)
ament_export_libraries(same_fuzzy_logic_controller)
ament_export_dependencies(${dependencies})

#pluginlib_export_plugin_description_file(nav2_core local_planner_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core same_fuzzy_logic_controller_plugin.xml)

ament_package()
