cmake_minimum_required(VERSION 3.5)
project(vreferee)

# 查找 ROS 2 包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(wust_interfaces REQUIRED)

# 添加可执行文件
add_executable(vreferee src/vreferee.cpp)

# 链接库
ament_target_dependencies(vreferee
  rclcpp
  wust_interfaces
)

# 安装可执行文件
install(TARGETS
  vreferee
  DESTINATION lib/${PROJECT_NAME}
)

# 导出目标
ament_package()
