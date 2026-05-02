cmake_minimum_required(VERSION 3.8)
project(camera_calibration)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY calibration calibration_files
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
