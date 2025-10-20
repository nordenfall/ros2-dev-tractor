cmake_minimum_required(VERSION 3.8)
project(kitti_writer)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

include_directories(include)

add_executable(kitti_writer_node
  src/kitti_writer.cpp
)

ament_target_dependencies(kitti_writer_node
  rclcpp sensor_msgs std_msgs tf2 tf2_ros tf2_sensor_msgs
)

install(TARGETS kitti_writer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
