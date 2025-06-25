cmake_minimum_required(VERSION 3.5)
project(yolo_landmark)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY launch config urdf sdf worlds
  DESTINATION share/yolo_landmark
)

install(
  PROGRAMS
    src/yolo_landmark/yolo_node.py
  DESTINATION lib/yolo_landmark
)

ament_package()
