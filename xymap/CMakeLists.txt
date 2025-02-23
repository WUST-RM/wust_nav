cmake_minimum_required(VERSION 3.8)
project(xymap)



# 查找所需的CMake包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)

# 添加可执行文件
add_executable(map_viewer src/map.cpp)

# 链接库
ament_target_dependencies(map_viewer
  rclcpp
  nav_msgs
  geometry_msgs
  std_srvs
  OpenCV
)

# 安装目标
install(TARGETS
  map_viewer
  DESTINATION lib/${PROJECT_NAME}
)

# # 安装启动文件（如果有）
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

# 导出依赖
ament_export_dependencies(
  rclcpp
  nav_msgs
  geometry_msgs
  std_srvs
  OpenCV
)

# 生成CMake配置
ament_package()
