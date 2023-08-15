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

#find_library(FUZZYLITE_LIBRARY
#  NAMES fuzzylite
#  HINTS "${CMAKE_INSTALL_PREFIX}/../fuzzylite/release/bin/")

nav2_package()

include_directories(
  include #rm fl/ and use the above instead
  ${CMAKE_INSTALL_PREFIX}/../fuzzylite/
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
        src/same_fuzzy_logic_controller/same_fuzzy_logic_controller.cpp
        src/same_fuzzy_logic_controller/collision_checker.cpp
        src/same_fuzzy_logic_controller/parameter_handler.cpp
        src/same_fuzzy_logic_controller/path_handler.cpp)

#================= dirty workaround until the above code is working
target_link_directories(${library_name} PUBLIC "/home/pgarcia/Dropbox/GitHub/same_fuzzy_logic_controller/fuzzylite/release/bin") #TODO ---------
target_link_libraries(${library_name} fuzzylite)



ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(line_follower_tester
  src/utils/line_follower_tester.cpp
)

ament_target_dependencies(line_follower_tester
  ${dependencies}
)

install(TARGETS ${library_name} line_follower_tester
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  #RUNTIME DESTINATION bin
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/
)

#target_link_libraries(${library_name} ${FUZZYLITE_LIBRARY})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  # add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core same_fuzzy_logic_controller_plugin.xml)

ament_package()
