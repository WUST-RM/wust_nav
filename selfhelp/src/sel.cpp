
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <queue>
#include <std_msgs/msg/detail/bool__struct.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <vector>
#include <mutex>
#include <cmath>
#include <limits>

using std::placeholders::_1;

class ClearanceNavigator : public rclcpp::Node
{
public:
  ClearanceNavigator()
  : Node("clearance_navigator"),
    executing_final_command_(false)
  { count_=0;
    this->declare_parameter("safe_threshold", 0.1);
    this->declare_parameter("recovery_speed", 1.0);
    this->declare_parameter("arrival_threshold", 0.1);
    this->declare_parameter("final_turn_angle", 6.28);
    this->declare_parameter("debug", false);
    debug_ = this->get_parameter("debug").as_bool();
    ifsafe=1;
    executing_final_command_=0;

    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "local_costmap/costmap", 10,
      std::bind(&ClearanceNavigator::costmapCallback, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", 10,
      std::bind(&ClearanceNavigator::odomCallback, this, _1));
    nav_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
        "navigate_to_pose/_action/status", 10,
        std::bind(&ClearanceNavigator::navStatusCallback, this, _1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    safe_pub_=this->create_publisher<std_msgs::msg::Bool>("safe_flag",10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ClearanceNavigator::controlLoop, this));

    costmap_received_ = false;
    odom_received_ = false;
  }

private:
  void navStatusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
  {
    navigation_active_ = false;
    for (const auto & status : msg->status_list) {
    if (status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
        status.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED) {
      navigation_active_ = true;
      break;
    }
  }
}
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    latest_costmap_ = msg;
    costmap_received_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_ = msg;
    odom_received_ = true;
  }

  std::vector<std::vector<double>> computeDistanceTransform(
    const nav_msgs::msg::OccupancyGrid & costmap)
  {
     int width = costmap.info.width;
          int height = costmap.info.height;
          double resolution = costmap.info.resolution;
      
          std::vector<std::vector<double>> dist(height, std::vector<double>(width, std::numeric_limits<double>::infinity()));
          std::queue<std::pair<int, int>> q;
      
          for (int j = 0; j < height; ++j) {
            for (int i = 0; i < width; ++i) {
              int index = j * width + i;
              int8_t cell_cost = costmap.data[index];
              if (cell_cost != 0) {  // 非0视为障碍
                dist[j][i] = 0.0;
                q.push(std::make_pair(i, j));
              }
            }
          }
      
          const std::vector<std::pair<int, int>> directions = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1},
            {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
          };
          const std::vector<double> step = {1.0, 1.0, 1.0, 1.0,
                                              std::sqrt(2), std::sqrt(2), std::sqrt(2), std::sqrt(2)};
      
          while (!q.empty()) {
            auto curr = q.front();
            q.pop();
            int cx = curr.first, cy = curr.second;
            double current_dist = dist[cy][cx];
            for (size_t d = 0; d < directions.size(); ++d) {
              int nx = cx + directions[d].first;
              int ny = cy + directions[d].second;
              if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                double new_dist = current_dist + step[d] * resolution;
                if (new_dist < dist[ny][nx]) {
                  dist[ny][nx] = new_dist;
                  q.push(std::make_pair(nx, ny));
                }
              }
            }
          }
          return dist;

  }

  void handleFinalCommand()
  {
    if (!odom_received_) {
      RCLCPP_WARN(this->get_logger(), "No odom data, skipping final command");
      return;
    }

    nav_msgs::msg::Odometry::SharedPtr odom;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      odom = latest_odom_;
    }

    const double final_turn_angle = this->get_parameter("final_turn_angle").as_double();
    const auto& current_twist = odom->twist.twist;
    const double vel_epsilon = 0.05;
    const double angular_epsilon = 0.05;

    bool linear_stopped = std::abs(current_twist.linear.x) < vel_epsilon &&
                          std::abs(current_twist.linear.y) < vel_epsilon;
    bool angular_reached = std::abs(current_twist.angular.z - final_turn_angle) < angular_epsilon;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.angular.z = final_turn_angle;

    if (linear_stopped && angular_reached) {
      executing_final_command_ = false;
      cmd.angular.z = final_turn_angle; 
      cmd_vel_pub_->publish(cmd);
      RCLCPP_INFO(this->get_logger(), "Final command succeeded, stopping");
    } else {
      cmd_vel_pub_->publish(cmd);
      if(debug_)
      {
      RCLCPP_INFO(this->get_logger(), "Issuing final command (angular: %.2f)", cmd.angular.z);}
      executing_final_command_ = false;
      ifsafe=1;
      std_msgs::msg::Bool safe;
      safe.data=ifsafe;
      safe_pub_->publish(safe);
    }
  }

  void controlLoop()
  { 
    if (navigation_active_) {
      if (debug_) {
        RCLCPP_INFO(this->get_logger(), "Navigation active, skipping control loop");
      }
      return;
    }
    if (executing_final_command_) {
      handleFinalCommand();
      return;
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_msg;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (!costmap_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No costmap received");
        return;
      }
      count_++;
      if(count_==1)
      {
        std::this_thread::sleep_for(std::chrono::seconds(5)); // 暂停5秒
      }

      costmap_msg = latest_costmap_;
    }

    const auto& costmap = *costmap_msg;
    const int width = costmap.info.width;
    const int height = costmap.info.height;
    const double resolution = costmap.info.resolution;
    const int robot_i = width / 2;
    const int robot_j = height / 2;

    const double safe_threshold = this->get_parameter("safe_threshold").as_double();
    const double recovery_speed = this->get_parameter("recovery_speed").as_double();
    const double arrival_threshold = this->get_parameter("arrival_threshold").as_double();
    const double final_turn_angle = this->get_parameter("final_turn_angle").as_double();

    auto distance_field = computeDistanceTransform(costmap);
    const double center_clearance = distance_field[robot_j][robot_i];

    // 如果当前已安全，重置状态并不发布命令
    if (center_clearance >= safe_threshold) {
     // executing_final_command_ = false;
     if(debug_)
     {
      RCLCPP_INFO(this->get_logger(), "Safe (clearance: %.2fm), no action", center_clearance);}
      if(ifsafe==0)
      {
        executing_final_command_ = true;
      }
      return;
    }

    // 寻找最近的安全目标
    double min_dist = std::numeric_limits<double>::infinity();
    int target_i = robot_i, target_j = robot_j;
    bool found = false;
    for (int j = 0; j < height; ++j) {
      for (int i = 0; i < width; ++i) {
        if (distance_field[j][i] >= safe_threshold) {
          const double dx = (i - robot_i) * resolution;
          const double dy = (j - robot_j) * resolution;
          const double d = std::hypot(dx, dy);
          if (d < min_dist) {
            min_dist = d;
            target_i = i;
            target_j = j;
            found = true;
          }
        }
      }
    }

    if (!found) {
      RCLCPP_WARN(this->get_logger(), "No safe target found");
      return;
    }

    // 检查是否到达目标
    const double vec_x = (target_i - robot_i) * resolution;
    const double vec_y = (target_j - robot_j) * resolution;
    const double norm = std::hypot(vec_x, vec_y);

    if (norm < arrival_threshold) {
      if(debug_)
      {
      RCLCPP_INFO(this->get_logger(), "Reached safe spot, initiating final command");}
      ifsafe = true;
      executing_final_command_ = true;
      handleFinalCommand();
    } else {
      // 发布移动命令
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = recovery_speed * (vec_x / norm);
      cmd.linear.y = recovery_speed * (vec_y / norm);
      cmd.angular.z = final_turn_angle;
      cmd_vel_pub_->publish(cmd);
      if(debug_)
      {
      RCLCPP_INFO(this->get_logger(), "Moving to safety (dist: %.2fm)", norm);}
      ifsafe = false;
      std_msgs::msg::Bool safe;
      safe.data=ifsafe;
      safe_pub_->publish(safe);
    }
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safe_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_status_sub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
  std::mutex costmap_mutex_;
  bool costmap_received_;

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  std::mutex odom_mutex_;
  bool odom_received_;

  int count_;


  bool executing_final_command_; 
  bool ifsafe;
  bool debug_;
  bool navigation_active_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClearanceNavigator>());
  rclcpp::shutdown();
  return 0;
}