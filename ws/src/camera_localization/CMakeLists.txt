cmake_minimum_required(VERSION 3.8)
project(camera_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(global_localization_node
  src/global_localization_node.cpp
)

ament_target_dependencies(global_localization_node
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  ament_index_cpp
)

target_include_directories(global_localization_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(global_localization_node
  ${OpenCV_LIBS}
)

install(TARGETS
  global_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()