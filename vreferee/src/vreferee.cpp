#include <chrono>
#include <iostream>
#include <memory>
#include <atomic>
#include <signal.h>
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "auto_aim_interfaces/msg/time_and_health.hpp"

using namespace std::chrono_literals;

class TimeAndHealthPublisher : public rclcpp::Node
{
public:
  TimeAndHealthPublisher(std::atomic<bool>& running)
  : Node("time_and_health_publisher"), running_(running)
  {
    // 声明参数，并设置默认值
    this->declare_parameter<int>("health", 600);
    this->declare_parameter<int>("time", 0);
    this->declare_parameter<int>("enable17mm", 600);
    this->declare_parameter<int>("score", 0);
    this->declare_parameter<int>("model", 0);
    // 获取初始参数值
    this->get_parameter("health", health_);
    this->get_parameter("time", time_);
    this->get_parameter("enable17mm", enable17mm_);
    this->get_parameter("score", score_);
    this->get_parameter("model", model_);

    // 添加参数更新回调，用于动态更新参数
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) -> rcl_interfaces::msg::SetParametersResult {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & param : parameters) {
          if (param.get_name() == "health") {
            health_ = param.as_int();
            RCLCPP_INFO(this->get_logger(), "Parameter 'health' changed to: %d", health_);
          } else if (param.get_name() == "time") {
            time_ = param.as_int();
            RCLCPP_INFO(this->get_logger(), "Parameter 'time' changed to: %d", time_);
          } else if (param.get_name() == "enable17mm") {
            enable17mm_ = param.as_int();
            RCLCPP_INFO(this->get_logger(), "Parameter 'enable17mm' changed to: %d", enable17mm_);
          } else if (param.get_name() == "score") {
            score_ = param.as_int();
            RCLCPP_INFO(this->get_logger(), "Parameter 'score' changed to: %d", score_);
          } else if (param.get_name() == "model") {
            model_ = param.as_int();
            RCLCPP_INFO(this->get_logger(), "Parameter 'model' changed to: %d", model_);
          }

        }
        return result;
      }
    );

    publisher_ = this->create_publisher<auto_aim_interfaces::msg::TimeAndHealth>("/referee", 10);
    timer_ = this->create_wall_timer(
      1s, std::bind(&TimeAndHealthPublisher::timer_callback, this));
  }

  void timer_callback()
  {
    if (!running_) return;
    
    auto message = auto_aim_interfaces::msg::TimeAndHealth();
    message.time = time_;
    message.health = health_;
    message.enable17mm = enable17mm_;
    message.score = score_;
    message.model = model_;
    RCLCPP_INFO(this->get_logger(), "Publishing: time=%d, health=%d, enable17mm=%d, score=%d, model=%d",  message.time, message.health,message.enable17mm,message.score,message.model);
    RCLCPP_INFO(this->get_logger(), "ros2 param set /time_and_health_publisher health/enable17mm/score/model");
    message.header.stamp = this->now();
    publisher_->publish(message);
    // 这里可以选择是否自动递增 time_，也可完全由参数控制
    time_++;
  }

private:
  rclcpp::Publisher<auto_aim_interfaces::msg::TimeAndHealth>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int time_;
  int health_;
  int enable17mm_;
  int score_;
  int model_;
  std::atomic<bool>& running_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

std::atomic<bool> running(true); 

void signal_handler(int signum) {
  if (signum == SIGINT) {
    std::cout << "\nCaught Ctrl+C, shutting down..." << std::endl;
    running = false;
  }
}

int main(int argc, char * argv[])
{
  signal(SIGINT, signal_handler);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TimeAndHealthPublisher>(running);

  // 直接使用 rclcpp::spin()，不再需要额外的输入线程
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
