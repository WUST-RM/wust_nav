#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <opencv2/opencv.hpp>
#include <mutex>
#include <thread>
#include <vector>

class MapViewer : public rclcpp::Node {
public:
  MapViewer() : Node("map_viewer"), single_point_mode_(true), has_single_point_(false) {
    // 设置QoS策略为TRANSIENT_LOCAL
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

    // 订阅/map话题
    map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", qos, std::bind(&MapViewer::mapCallback, this, std::placeholders::_1));

    // 创建发布器
    single_point_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("single_point_coords", 10);
    four_point_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("four_point_coords", 10);

    // 创建OpenCV窗口
    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    cv::setMouseCallback(window_name_, onMouse, this);
  }

  std::mutex map_mutex_;
  cv::Mat map_image_;
  nav_msgs::msg::MapMetaData map_info_;

  // 坐标转换函数
  cv::Point2i worldToPixel(double world_x, double world_y) {
    double dx = world_x - map_info_.origin.position.x;
    double dy = world_y - map_info_.origin.position.y;
    int px = static_cast<int>(dx / map_info_.resolution);
    int py = static_cast<int>(dy / map_info_.resolution);
    return cv::Point2i(px, map_info_.height - py - 1);
  }

  // 鼠标回调处理
  static void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (event != cv::EVENT_LBUTTONDOWN) return;
    MapViewer* viewer = reinterpret_cast<MapViewer*>(userdata);
    viewer->processMouseClick(x, y);
  }

  void processMouseClick(int x, int y) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (map_image_.empty()) return;

    // 计算地图坐标
    double map_x = map_info_.origin.position.x + x * map_info_.resolution;
    double map_y = map_info_.origin.position.y + (map_info_.height - y - 1) * map_info_.resolution;

    if (single_point_mode_) {
      current_single_point_.x = map_x;
      current_single_point_.y = map_y;
      has_single_point_ = true;
    } else {
      if (current_four_points_.size() < 4) {
        current_four_points_.emplace_back(map_x, map_y);
      }
    }
  }

  // 发布函数
  void publishSinglePoint() {
    auto msg = geometry_msgs::msg::Point();
    msg.x = current_single_point_.x;
    msg.y = current_single_point_.y;
    single_point_publisher_->publish(msg);
  }

  void publishFourPoints() {
    std_msgs::msg::Float32MultiArray msg;
    for (const auto& p : current_four_points_) {
      msg.data.push_back(p.first);
      msg.data.push_back(p.second);
    }
    four_point_publisher_->publish(msg);
  }

  // 成员变量
  const std::string window_name_ = "Map Viewer";
  bool single_point_mode_;
  geometry_msgs::msg::Point current_single_point_;
  bool has_single_point_;
  std::vector<std::pair<double, double>> current_four_points_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr single_point_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr four_point_publisher_;

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_info_ = msg->info;
    int width = msg->info.width;
    int height = msg->info.height;

    cv::Mat img(height, width, CV_8UC1);
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        int8_t value = msg->data[i * width + j];
        img.at<uint8_t>(height - i - 1, j) = value == -1 ? 127 : (value == 0 ? 255 : 0);
      }
    }
    map_image_ = img;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapViewer>();
  std::thread spin_thread([node]() { rclcpp::spin(node); });

  while (rclcpp::ok()) {
    cv::Mat display;
    {
      std::lock_guard<std::mutex> lock(node->map_mutex_);
      if (node->map_image_.empty()) continue;

      // 创建彩色图像并绘制元素
      cv::cvtColor(node->map_image_, display, cv::COLOR_GRAY2BGR);

      // 绘制原点（红色）
      auto origin = node->worldToPixel(0.0, 0.0);
if (origin.x >= 0 && origin.x < display.cols && origin.y >= 0 && origin.y < display.rows) {
  cv::circle(display, origin, 1, cv::Scalar(0, 0, 255), 1); // 半径1，厚度1
}

// 绘制单点（绿色）
if (node->single_point_mode_ && node->has_single_point_) {
  auto pt = node->worldToPixel(node->current_single_point_.x, node->current_single_point_.y);
  if (pt.x >= 0 && pt.x < display.cols && pt.y >= 0 && pt.y < display.rows) {
    cv::circle(display, pt, 1, cv::Scalar(0, 255, 0), 1); // 半径1，厚度1
  }
}

// 绘制四点（蓝色）
if (!node->single_point_mode_) {
  for (const auto& p : node->current_four_points_) {
    auto pt = node->worldToPixel(p.first, p.second);
    if (pt.x >= 0 && pt.x < display.cols && pt.y >= 0 && pt.y < display.rows) {
      cv::circle(display, pt, 1, cv::Scalar(255, 0, 0), 1); // 半径1，厚度1
    }
  }
}
      
    }

    cv::imshow(node->window_name_, display);
    int key = cv::waitKey(30);

    // 处理按键
    if (key == 27) break; // ESC退出
    else if (key == 'm') { // 切换模式
      node->single_point_mode_ = !node->single_point_mode_;
      RCLCPP_INFO(node->get_logger(), "切换到%s模式", node->single_point_mode_ ? "单点" : "四点");
      std::lock_guard<std::mutex> lock(node->map_mutex_);
      if (node->single_point_mode_) node->current_four_points_.clear();
      else node->has_single_point_ = false;
    }
    else if (key == 8 || key == 127) { // 退格键
      std::lock_guard<std::mutex> lock(node->map_mutex_);
      if (node->single_point_mode_) {
        node->has_single_point_ = false;
      } else {
        if (!node->current_four_points_.empty()) {
          node->current_four_points_.pop_back();
        }
      }
    }
    else if (key == 13) { // 回车键发布
      std::lock_guard<std::mutex> lock(node->map_mutex_);
      if (node->single_point_mode_) {
        if (node->has_single_point_) {
          node->publishSinglePoint();
          node->has_single_point_ = false;
        }
      } else {
        if (node->current_four_points_.size() == 4) {
          node->publishFourPoints();
          node->current_four_points_.clear();
        }
      }
    }
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}