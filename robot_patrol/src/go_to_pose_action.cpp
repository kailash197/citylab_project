#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/utils.h"
#include <cmath>

using Interface_GoToPose = robot_patrol::action::GoToPose;
using Pose2D = geometry_msgs::msg::Pose2D;
using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<Interface_GoToPose>;
using namespace std::placeholders;

class GoToPose : public rclcpp::Node {
private:
    rclcpp_action::Server<Interface_GoToPose>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    
    Pose2D desired_pos_;
    Pose2D current_pos_;
    int direction_ = 1;

    void odom_topic_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid, 
        std::shared_ptr<const Interface_GoToPose::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle);
    void execute_action(const std::shared_ptr<GoalHandleGoToPose> goal_handle);
    void stop_robot();

public:
    explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()):
    Node("gotopose_action_server", options) {
        action_server_ = rclcpp_action::create_server<Interface_GoToPose>(
            this, "go_to_pose",
            std::bind(&GoToPose::handle_goal, this, _1, _2),
            std::bind(&GoToPose::handle_cancel, this, _1),
            std::bind(&GoToPose::handle_accepted, this, _1));
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&GoToPose::odom_topic_callback, this,_1));
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);        
        RCLCPP_INFO(this->get_logger(), "Action Server Ready");
    }
};

// Function to calculate Euclidean distance between two Pose2D objects
double calculate_distance(const Pose2D &p1, const Pose2D &p2)
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Function to normalize angle to [0, 2PI]
double normalize_angle(double angle) {
    // Normalize the angular velocity between -pi and pi
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// Calculate angular velocity based on current and desired positions
double calc_angular_vel(const Pose2D &desired, const Pose2D &current)
{
    double angle_to_goal = atan2(desired.y - current.y, desired.x - current.x);
    double angular_vel = angle_to_goal - current.theta;

    // Normalize the angular velocity between -pi and pi
    angular_vel = normalize_angle(angular_vel);

    return angular_vel;
}

void GoToPose::odom_topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;
    current_pos_.theta = normalize_angle(tf2::getYaw(q));
}

rclcpp_action::GoalResponse GoToPose::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Interface_GoToPose::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Action Called");
    (void)uuid;
    desired_pos_ = goal->goal_pos;
    desired_pos_.theta = normalize_angle(desired_pos_.theta);
    this->direction_ = (desired_pos_.theta > 0) ? 1 : -1;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GoToPose::handle_cancel(
    const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Action Cancelled");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void GoToPose::handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    std::thread{std::bind(&GoToPose::execute_action, this, _1), goal_handle}.detach();
}

void GoToPose::execute_action(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(7); // 7 Hz
    auto feedback = std::make_shared<Interface_GoToPose::Feedback>();
    auto result = std::make_shared<Interface_GoToPose::Result>();
    geometry_msgs::msg::Twist move_cmd_vel;

    feedback->current_pos = this->current_pos_;
    move_cmd_vel.linear.x = 0.2; //constant
    while( rclcpp::ok() && calculate_distance(desired_pos_, current_pos_) > 0.03){
        // Check if there is a cancel request
        if (goal_handle->is_canceling()) {
            result->status = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Action canceled");
            return;
        }
        move_cmd_vel.angular.z = calc_angular_vel(desired_pos_, current_pos_);
        cmd_vel_publisher_->publish(move_cmd_vel);

        feedback->current_pos = current_pos_; // Update feedback with current position
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback: (%.2f, %.2f, %.2f)",
        current_pos_.x, current_pos_.y, current_pos_.theta);
        loop_rate.sleep();
    }
    //now stop and adjust theta orientation
    this->stop_robot();


    double error = fabs(normalize_angle(desired_pos_.theta - current_pos_.theta));
    while(rclcpp::ok() && error >= 0.01){
        // Adjust angular speed based on the error
        move_cmd_vel.linear.x = 0.0;
        if (error < 0.5) {
            move_cmd_vel.angular.z = error * this->direction_;
        } else {
            move_cmd_vel.angular.z = 0.2 * this->direction_;
        }
        cmd_vel_publisher_->publish(move_cmd_vel);
        
        feedback->current_pos = current_pos_; // Update feedback with current position
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback: (%.2f, %.2f, %.2f)",
        current_pos_.x, current_pos_.y, current_pos_.theta);
        error = fabs(normalize_angle(desired_pos_.theta - current_pos_.theta));
        loop_rate.sleep();
    }
    //now stop
    this->stop_robot();

    // Check if goal is done
    if (rclcpp::ok()) {
        result->status = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Action Completed");
    }
}

void GoToPose::stop_robot(){
    geometry_msgs::msg::Twist move_cmd_vel;
    move_cmd_vel.linear.x = 0.0;
    move_cmd_vel.angular.z = 0.0;
    cmd_vel_publisher_->publish(move_cmd_vel);
    rclcpp::sleep_for(std::chrono::milliseconds(400));
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<GoToPose>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(action_server);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
