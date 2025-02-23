#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <auto_aim_interfaces/msg/time_and_health.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "geometry_msgs/msg/point32.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>
#include "auto_aim_interfaces/msg/time_and_health.hpp"
#include <functional>
#include <vector>


// struct Goal {
//     double x;
//     double y;
// };
class DataLogger : public rclcpp::Node {
public:
  DataLogger()
  : Node("data_logger_node"),
    four_point_counter_(0),
    single_point_counter_(0),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(*tf_buffer_)
  {
    // 订阅四个坐标点
    four_point_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/four_point_coords", 10,
      std::bind(&DataLogger::fourPointCallback, this, std::placeholders::_1));

    // 订阅单点坐标
    single_point_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/single_point_coords", 10,
      std::bind(&DataLogger::singlePointCallback, this, std::placeholders::_1));

    // 订阅时间和健康状态
    time_health_sub_ = this->create_subscription<auto_aim_interfaces::msg::TimeAndHealth>(
      "/time_and_health", 10,
      std::bind(&DataLogger::timeHealthCallback, this, std::placeholders::_1));

    // 初始化导航目标的 action 客户端
    navigate_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, "/navigate_to_pose");

    // 初始化通过多个姿态导航的 action 客户端
    navigate_through_poses_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
      this, "/navigate_through_poses");

    // 定时器，用于定期获取机器人在 map 坐标系中的位置
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&DataLogger::getRobotPose, this));
  }

private:
  // 四个坐标点的回调函数
  void fourPointCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    four_point_counter_++;
    RCLCPP_INFO(this->get_logger(), "------ Four Points #%d ------", four_point_counter_);
    
    if (msg->data.size() == 8) {
      RCLCPP_INFO(this->get_logger(), 
                 "Points: (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f)",
                 msg->data[0], msg->data[1],
                 msg->data[2], msg->data[3],
                 msg->data[4], msg->data[5],
                 msg->data[6], msg->data[7]);
    }
    all_four_points_.push_back(*msg);
  }

  // 单点坐标的回调函数
  void singlePointCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
    single_point_counter_++;
    RCLCPP_INFO(this->get_logger(), "------ Single Point #%d ------", single_point_counter_);
    RCLCPP_INFO(this->get_logger(), "Position: (%.2f, %.2f, %.2f)",
               msg->x, msg->y, msg->z);
    all_single_points_.push_back(*msg);

    // 发布导航目标
   // sendNavigateToPoseGoal(msg->x, msg->y, msg->z);
  }

  // 时间和健康状态的回调函数
  void timeHealthCallback(const auto_aim_interfaces::msg::TimeAndHealth::SharedPtr msg) {
    latest_time_health_ = *msg;
  }

  // 发送导航目标的函数
  void sendNavigateToPoseGoal(float x, float y) {
    if (!navigate_to_pose_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available");
      return;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    //goal_msg.pose.pose.position.z = z;
    goal_msg.pose.pose.orientation.w = 1.0; // 默认朝向

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Navigation failed");
      }
    };

    navigate_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
  }
  void navigate_through_poses_(std::vector<geometry_msgs::msg::PoseStamped> goals) {
  // 检查 action 服务器是否可用
  if (!navigate_through_poses_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(), "NavigateThroughPoses action server not available");
    return;
  }

  // 设置目标消息
  auto goal_msg = nav2_msgs::action::NavigateThroughPoses::Goal();
  goal_msg.poses = goals;

  // 设置发送目标的选项
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
  send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult & result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Navigation through poses succeeded");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Navigation through poses failed");
    }
  };

  // 异步发送目标
  navigate_through_poses_client_->async_send_goal(goal_msg, send_goal_options);
}
bool ifin(double x, double y, const std_msgs::msg::Float32MultiArray& region) {
  // 假设region.data中存储了四个点的坐标，顺序为x1, y1, x2, y2, x3, y3, x4, y4
  if (region.data.size() != 8) {
      return false; // 如果数据格式不正确，返回false
  }

  std::vector<geometry_msgs::msg::Point32> polygon;
  for (size_t i = 0; i < region.data.size(); i += 2) {
      geometry_msgs::msg::Point32 point;
      point.x = region.data[i];
      point.y = region.data[i + 1];
      polygon.push_back(point);
  }

  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
      if (((polygon[i].y > y) != (polygon[j].y > y)) &&
          (x < (polygon[j].x - polygon[i].x) * (y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x)) {
          inside = !inside;
      }
  }

  return inside;
}
  // 获取机器人在 map 坐标系中的位置
  void getRobotPose() {
    try {
      auto transform_stamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      RCLCPP_INFO(this->get_logger(), "Robot Position in map frame: (%.2f, %.2f, %.2f)",
                 transform_stamped.transform.translation.x,
                 transform_stamped.transform.translation.y,
                 transform_stamped.transform.translation.z);
      thisx=transform_stamped.transform.translation.x;
      thisy=transform_stamped.transform.translation.y; 
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform 'map' to 'base_link': %s", ex.what());
    }

  }

  // 订阅器
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr four_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr single_point_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::TimeAndHealth>::SharedPtr time_health_sub_;

  // 计数器
  int four_point_counter_;
  std::vector<std_msgs::msg::Float32MultiArray> all_four_points_;

  int single_point_counter_;
  std::vector<geometry_msgs::msg::Point> all_single_points_;

  // 状态存储
  auto_aim_interfaces::msg::TimeAndHealth latest_time_health_;

  // Action 客户端
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr navigate_through_poses_client_;

  // TF 相关
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;

  double thisx;
  double thisy;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataLogger>());
  rclcpp::shutdown();
  return 0;
}