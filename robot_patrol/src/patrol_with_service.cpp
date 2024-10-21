#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

constexpr double LINEAR_SPEED = 0.1;

class PatrolClient : public rclcpp::Node {
public:
  PatrolClient() : Node("robot_client_node") {
    // Create callback groups
    timer_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    laser_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = laser_cb_group_;

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&PatrolClient::laser_scan_callback, this, std::placeholders::_1),
        subscription_options);
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&PatrolClient::timer_callback, this), timer_cb_group_);
    directionServiceClient_ = this->create_client<robot_patrol::srv::GetDirection>("direction_service"); 
    RCLCPP_INFO(this->get_logger(), "Patrol Node Service Client Ready");
  }

private:
  void
  laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  void timer_callback();
  void client_response_callback(rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr directionServiceClient_;
  
  sensor_msgs::msg::LaserScan::SharedPtr scan_message;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::CallbackGroup::SharedPtr laser_cb_group_;
  bool request_in_progress = false;
};

void PatrolClient::laser_scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr scan_message) {
    this->scan_message = scan_message;
    RCLCPP_INFO(this->get_logger(), "Laser Scan Called.");
}

void PatrolClient::timer_callback() {
  if (request_in_progress) {
    RCLCPP_WARN(this->get_logger(), "Previous request still in progress. Skipping...");
    return;
  }
  auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();  

  while (!directionServiceClient_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the direction service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Direction service not available, waiting again...");
  }

  request->laser_data = *scan_message;

  request_in_progress = true;

  auto result_future = directionServiceClient_->async_send_request(
    request, std::bind(&PatrolClient::client_response_callback, this,
    std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Service Request: Sent to direction service");
}

void PatrolClient::client_response_callback(rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future){
  auto result = future.get();
  auto command = geometry_msgs::msg::Twist();
  command.linear.x = LINEAR_SPEED;
  if (result->direction == "forward"){
    command.angular.z = 0.0;
  } else if (result->direction == "left") {
    command.angular.z = 0.5;
  } else if (result->direction == "right") {
    command.angular.z = -0.5;
  } else {
    command.linear.x = 0.0;
    RCLCPP_ERROR(this->get_logger(), "Service Response: Unknown %s", result->direction.c_str());
  }
  publisher_->publish(command);
  request_in_progress = false;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto patrol_node = std::make_shared<PatrolClient>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(patrol_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
