cmake_minimum_required(VERSION 3.5)
project(regulated_fuzzy_logic_controller)

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

#message(WARNING ${PROJECT_SOURCE_DIR}/../3rd_party/fuzzylite/build/bin/)
find_library(FUZZYLITE_LIBRARY
  NAMES fuzzylite
  HINTS "${CMAKE_INSTALL_PREFIX}/../fuzzylite/lib/") 

nav2_package()

include_directories(
  include
  ${CMAKE_INSTALL_PREFIX}/../fuzzylite/include/
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
set(library_name regulated_fuzzy_logic_controller)

add_library(${library_name} SHARED
        src/regulated_fuzzy_logic_controller/regulated_fuzzy_logic_controller.cpp
        src/regulated_fuzzy_logic_controller/collision_checker.cpp
        src/regulated_fuzzy_logic_controller/parameter_handler.cpp
        src/regulated_fuzzy_logic_controller/path_handler.cpp)

target_link_libraries(${library_name} ${FUZZYLITE_LIBRARY})

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core regulated_fuzzy_logic_controller_plugin.xml)

ament_package()
