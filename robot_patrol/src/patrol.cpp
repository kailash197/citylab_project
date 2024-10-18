#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

constexpr double OBSTACLE_LIMIT = 0.35;
constexpr double LINEAR_SPEED = 0.1;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("robot_patrol_node") {
    // Create callback groups
    timer_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    laser_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = laser_cb_group_;

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&Patrol::laser_scan_callback, this, std::placeholders::_1),
        subscription_options);
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&Patrol::timer_callback, this), timer_cb_group_);
    RCLCPP_INFO(this->get_logger(), "Patrol Commander Created.");
  }

private:
  void
  laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  void timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::CallbackGroup::SharedPtr laser_cb_group_;
  bool front_obstacle = false;
  bool left_obstacle = false;
  bool right_obstacle = false;
  double direction_;
};

void Patrol::laser_scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr scan_message) {
  front_obstacle = false;
  left_obstacle = false;
  right_obstacle = false;

  // Transform radian angle to indices
  int length = scan_message->ranges.size();
  double angle_min = scan_message->angle_min;
  double angle_max = scan_message->angle_max;

  auto transform_index = [&](double angle) {
    return static_cast<int>((length * (angle - angle_min)) /
                            (angle_max - angle_min));
  };
  int start = transform_index(-M_PI_2), end = transform_index(M_PI_2);
  int start_right = start, end_right = transform_index(-M_PI / 6);
  int start_front = end_right, end_front = transform_index(M_PI / 6);
  int start_left = end_front, end_left = end;

  int obstacle_count_threshold = 10;
  // Minimum number of rays below OBSTACLE_LIMIT to consider an obstacle
  std::string obstacle_directions = "";

  // Check for front obstacle
  int front_count = 0;
  for (int i = start_front; i <= end_front; ++i) {
    if (scan_message->ranges[i] < OBSTACLE_LIMIT) {
      front_count++;
    }
  }
  if (front_count >= obstacle_count_threshold) {
    front_obstacle = true;
    obstacle_directions += "Front ";
  }

  // Check for left obstacle
  int left_count = 0;
  for (int i = start_left; i <= end_left; ++i) {
    if (scan_message->ranges[i] < OBSTACLE_LIMIT) {
      left_count++;
    }
  }
  if (left_count >= obstacle_count_threshold) {
    left_obstacle = true;
    obstacle_directions += "Left ";
  }

  // Check for right obstacle
  int right_count = 0;
  for (int i = start_right; i <= end_right; ++i) {
    if (scan_message->ranges[i] < OBSTACLE_LIMIT) {
      right_count++;
    }
  }
  if (right_count >= obstacle_count_threshold) {
    right_obstacle = true;
    obstacle_directions += "Right ";
  }

  double largest_distance = 0.0;
  for (int i = start; i < end; i++) {
    if (std::isfinite(scan_message->ranges[i]) &&
        scan_message->ranges[i] > largest_distance) {
      largest_distance = scan_message->ranges[i];
      direction_ = -(M_PI / 2) + (i - start) * M_PI / (end - start);
    }
  }
}

void Patrol::timer_callback() {
  auto command = geometry_msgs::msg::Twist();
  command.linear.x = LINEAR_SPEED;
  if (!front_obstacle) {
    command.angular.z = 0.0;
  } else {
    command.angular.z = direction_ / 2.0;
  }
  publisher_->publish(command);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto patrol_node = std::make_shared<Patrol>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(patrol_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
