#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

constexpr double OBSTACLE_LIMIT = 0.35;

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
  }

private:
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void timer_callback();
  double avg_distance(double start_angle_deg, double end_angle_deg,
                      const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::CallbackGroup::SharedPtr laser_cb_group_;

  bool front_obstacle = false;
  bool left_obstacle = false;
  bool right_obstacle = false;
};

void Patrol::laser_scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  std::string obstacle_directions = "";
  if (avg_distance(-90, -60, msg) < OBSTACLE_LIMIT) {
    right_obstacle = true;
    obstacle_directions += "Right ";
  }
  if (avg_distance(-10, 10, msg) < OBSTACLE_LIMIT) {
    front_obstacle = true;
    obstacle_directions += "Front ";
  }
  if (avg_distance(60, 90, msg) < OBSTACLE_LIMIT) {
    left_obstacle = true;
    obstacle_directions += "Left ";
  }

  if (!obstacle_directions.empty()) {
    RCLCPP_INFO(this->get_logger(), "Obstacles detected at: %s",
                obstacle_directions.c_str());
  }
}

void Patrol::timer_callback() {
  auto message = geometry_msgs::msg::Twist();
  message.linear.x = 0.1;
  message.angular.z = -0.2;
  publisher_->publish(message);
}

double Patrol::avg_distance(double start_angle_deg, double end_angle_deg,
                            const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Convert degrees to radians
  double start_angle = start_angle_deg * M_PI / 180.0;
  double end_angle = end_angle_deg * M_PI / 180.0;

  // Make sure angles are within the bounds of the scan
  start_angle = std::max(start_angle, static_cast<double>(msg->angle_min));

  end_angle = std::min(end_angle, static_cast<double>(msg->angle_max));

  // Get the indices for start and end angles
  int start_index =
      static_cast<int>((start_angle - msg->angle_min) / msg->angle_increment);
  int end_index =
      static_cast<int>((end_angle - msg->angle_min) / msg->angle_increment);

  double sum = 0.0;
  int count = 0;

  // Loop through the relevant ranges
  for (int i = start_index; i <= end_index; ++i) {
    double distance = msg->ranges[i];
    // Check if the distance is within valid range
    if (distance >= msg->range_min && distance <= msg->range_max) {
      sum += distance;
      count++;
    }
  }

  // If no valid points, return -1 or some error indicator
  if (count == 0)
    return -1.0;

  // Return the average distance
  return sum / count;
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
