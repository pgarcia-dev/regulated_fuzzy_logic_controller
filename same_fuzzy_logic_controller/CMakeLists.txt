cmake_minimum_required(VERSION 3.5)
project(same_fuzzy_logic_controller)


find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(navigation2 REQUIRED)

nav2_package()

include_directories(
  include
)


set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
  tf2_geometry_msgs
)
set(library_name same_fuzzy_logic_controller)

add_library(${library_name} SHARED
        src/same_fuzzy_logic_controller.cpp
        src/collision_checker.cpp
        src/parameter_handler.cpp
        src/path_handler.cpp)

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(DIRECTORY include/
  DESTINATION include/
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

target_link_directories(${library_name} PUBLIC "/home/pgarcia/fuzzylite/fuzzylite/release/bin")#---------
target_link_libraries(${library_name} fuzzylite)

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
 # set(ament_cmake_copyright_FOUND TRUE)
 # ament_lint_auto_find_test_dependencies()
 # add_subdirectory(test)
#endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

#pluginlib_export_plugin_description_file(nav2_core local_planner_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core same_fuzzy_logic_controller_plugin.xml)

ament_package()
