cmake_minimum_required(VERSION 3.8)
project(match_nav2_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(match_nav2_wait_for_match_bt_node SHARED
  src/wait_for_match_condition.cpp
)

target_include_directories(match_nav2_wait_for_match_bt_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_compile_definitions(match_nav2_wait_for_match_bt_node PRIVATE BT_PLUGIN_EXPORT)

ament_target_dependencies(match_nav2_wait_for_match_bt_node
  behaviortree_cpp
  rclcpp
  std_msgs
)

install(TARGETS match_nav2_wait_for_match_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
