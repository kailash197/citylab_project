#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

constexpr double LINEAR_SPEED = 0.1;

class TestService : public rclcpp::Node {
public:
TestService() : Node("robot_patrol_node") {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&TestService::laser_scan_callback, this, std::placeholders::_1));
    directionServiceClient_ = this->create_client<robot_patrol::srv::GetDirection>("direction_service"); 
    RCLCPP_INFO(this->get_logger(), "Service Client Ready");
}

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr directionServiceClient_;
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg); 
    void client_response_callback(rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future);
};

void TestService::laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_message){
    auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
    while (!directionServiceClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the direction service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Direction service not available, waiting again...");
    }

    request->laser_data = *scan_message;
    auto result_future = directionServiceClient_->async_send_request(
        request, std::bind(&TestService::client_response_callback, this,
        std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Service Request: Sent to direction service");
}

void TestService::client_response_callback(rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture result_future){
    auto result = result_future.get();
    RCLCPP_INFO(this->get_logger(), "Service Response: Direction -> %s", result->direction.c_str());
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto test_node = std::make_shared<TestService>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(test_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
