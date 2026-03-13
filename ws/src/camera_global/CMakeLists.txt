cmake_minimum_required(VERSION 3.16)
project(simulation CXX)
set(CMAKE_CXX_STANDARD 17)

set(OpenCV_DIR "/opt/opencv-4.12/lib/cmake/opencv4")
find_package(OpenCV 4.12 REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)


# Perception node -----------------------------------------------
add_executable(perception_node src/perception_node.cpp)
ament_target_dependencies(perception_node
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(perception_node ${OpenCV_LIBS} cv_bridge::cv_bridge) # Link OpenCV the regular way
# ---------------------------------------------------------
# Task planning node -----------------------------------------------
# add_executable(task_planner_node src/task_planner_node.cpp)
# ament_target_dependencies(task_planner_node
#   rclcpp
#   sensor_msgs
#   cv_bridge
# )
# target_link_libraries(perception_node ${OpenCV_LIBS} cv_bridge::cv_bridge) # Link OpenCV the regular way
# ---------------------------------------------------------
# Simulation perception node -----------------------------------------
add_executable(perception_sim_node src/perception_sim_node.cpp)
ament_target_dependencies(perception_sim_node
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(perception_sim_node ${OpenCV_LIBS} cv_bridge::cv_bridge) # Link OpenCV the regular way
# ----------------------------------------------------------