cmake_minimum_required(VERSION 3.16)
project(non_groundPCD)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Open3D REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(non_ground src/non_ground.cpp)
target_include_directories(non_ground PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(non_ground
  rclcpp
  sensor_msgs
  pcl_conversions
  Open3D
  Eigen3
)

install(TARGETS non_ground
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
launch config rviz
DESTINATION share/${PROJECT_NAME})
ament_package()