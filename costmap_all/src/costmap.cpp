#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <memory>
#include <algorithm>

using nav_msgs::msg::OccupancyGrid;
using geometry_msgs::msg::Pose;

class CostmapFusion : public rclcpp::Node {
public:
  CostmapFusion() 
    : Node("costmap_fusion_node"), 
      tf_buffer_(this->get_clock()), 
      tf_listener_(tf_buffer_) 
  {
    // 订阅全局和局部代价地图
    global_sub_ = create_subscription<OccupancyGrid>(
      "/global_costmap/costmap", 1,
      [this](const OccupancyGrid::SharedPtr msg) { global_map_ = msg; });
    
    local_sub_ = create_subscription<OccupancyGrid>(
      "/local_costmap/costmap", 1,
      [this](const OccupancyGrid::SharedPtr msg) { local_map_ = msg; });

    // 发布融合后的代价地图
    merged_pub_ = create_publisher<OccupancyGrid>("/merged_costmap", 1);

    // 定时器处理融合和发布
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        if (global_map_ && local_map_) {
          auto merged = std::make_unique<OccupancyGrid>();
          if (fuse_costmaps(*global_map_, *local_map_, *merged)) {
            merged_pub_->publish(std::move(merged));
          }
        }
      });
  }

private:
  bool fuse_costmaps(const OccupancyGrid& global, const OccupancyGrid& local, OccupancyGrid& merged) {
    // 获取从局部地图到全局地图的变换
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        global.header.frame_id, 
        local.header.frame_id, 
        tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "Failed to transform local costmap: %s", ex.what());
      return false;
    }

    // 变换局部地图的原点到全局坐标系
    geometry_msgs::msg::Pose transformed_local_origin;
    tf2::doTransform(local.info.origin, transformed_local_origin, transform);

    // 使用变换后的原点进行融合
    return fuse_costmaps_with_transform(global, local, merged, transformed_local_origin);
  }

  bool fuse_costmaps_with_transform(const OccupancyGrid& global, const OccupancyGrid& local, OccupancyGrid& merged, const Pose& local_origin) {
    // 保留全局地图参数
    merged.header = global.header;
    merged.info = global.info;
    merged.data = global.data;

    // 获取地图参数
    const double global_res = global.info.resolution;
    const double local_res = local.info.resolution;
    const Pose& global_origin = global.info.origin;

    // 遍历全局地图每个单元格
    for (size_t gy = 0; gy < global.info.height; ++gy) {
      for (size_t gx = 0; gx < global.info.width; ++gx) {
        // 计算全局单元格的世界坐标
        const double x_world = global_origin.position.x + gx * global_res;
        const double y_world = global_origin.position.y + gy * global_res;

        // 转换到局部地图坐标系
        const double local_x = (x_world - local_origin.position.x) / local_res;
        const double local_y = (y_world - local_origin.position.y) / local_res;

        // 获取最近的局部地图索引
        const int lx = static_cast<int>(std::round(local_x));
        const int ly = static_cast<int>(std::round(local_y));

        // 检查边界
        if (lx >= 0 && lx < static_cast<int>(local.info.width) &&
            ly >= 0 && ly < static_cast<int>(local.info.height)) 
        {
          // 获取并融合数据
          const size_t global_idx = gy * global.info.width + gx;
          const size_t local_idx = ly * local.info.width + lx;
          const int8_t local_value = local.data[local_idx];
          
          // 有效值处理（忽略-1未知区域）
          if (local_value >= 0) {
            merged.data[global_idx] = std::max(
              merged.data[global_idx],
              static_cast<int8_t>(local_value)
            );
          }
        }
      }
    }
    return true;
  }

  rclcpp::Subscription<OccupancyGrid>::SharedPtr global_sub_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr local_sub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr merged_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  OccupancyGrid::SharedPtr global_map_;
  OccupancyGrid::SharedPtr local_map_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapFusion>());
  rclcpp::shutdown();
  return 0;
}