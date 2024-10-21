#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

constexpr double OBSTACLE_LIMIT = 0.35;
constexpr double LINEAR_SPEED = 0.1;
constexpr int obstacle_count_threshold = 10;
constexpr auto timer_period = 100ms;

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
        timer_period, std::bind(&PatrolClient::timer_callback, this), timer_cb_group_);
    directionServiceClient_ = this->create_client<robot_patrol::srv::GetDirection>("direction_service"); 
    RCLCPP_INFO(this->get_logger(), "Patrol Service Client Ready");
  }

private:
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  void timer_callback();
  void call_direction_service();
  void client_response_callback(rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr directionServiceClient_;
  
  sensor_msgs::msg::LaserScan::SharedPtr scan_message;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::CallbackGroup::SharedPtr laser_cb_group_;
  bool front_obstacle = false;
  bool request_in_progress = false;
  std::string direction_ = "forward";
};

void PatrolClient::laser_scan_callback( const sensor_msgs::msg::LaserScan::SharedPtr scan_message) {
  this->scan_message = scan_message;
  front_obstacle = false;

  // Transform radian angle to indices
  int length = scan_message->ranges.size();
  double angle_min = scan_message->angle_min;
  double angle_max = scan_message->angle_max;

  auto transform_index = [&](double angle) {
    return static_cast<int>((length * (angle - angle_min)) /
                            (angle_max - angle_min));
  };
  int start_front = transform_index(-M_PI / 6), end_front = transform_index(M_PI / 6);
  
  // OBSTACLE_LIMIT: Minimum number of rays to consider an obstacle
  // Check for obstacle at front
  int front_count = 0;
  for (int i = start_front; i <= end_front; ++i) {
    if (scan_message->ranges[i] < OBSTACLE_LIMIT) {
      front_count++;
    }
    if (front_count >= obstacle_count_threshold) {
      front_obstacle = true;
      break;
    }
  }
}

void PatrolClient::timer_callback() {
  auto command = geometry_msgs::msg::Twist();
  command.linear.x = LINEAR_SPEED;
  if (!front_obstacle) {
    command.angular.z = 0.0;
    publisher_->publish(command);
    return;
  } else {
    call_direction_service();
  }
  
  if (direction_ == "left") {
    command.angular.z = 0.5;
  } else if (direction_ == "right") {
    command.angular.z = -0.5;
  } else if (direction_ == "forward"){
    command.angular.z = 0.0;
  }  
  publisher_->publish(command);
}

void PatrolClient::call_direction_service() {
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
  direction_ = result->direction;
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
