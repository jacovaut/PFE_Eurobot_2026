cmake_minimum_required(VERSION 3.8)
project(camera_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(fake_camera_localization_node src/fake_camera_localization_node.cpp)

ament_target_dependencies(fake_camera_localization_node
  rclcpp
  geometry_msgs
  tf2
)

install(TARGETS
  fake_camera_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()