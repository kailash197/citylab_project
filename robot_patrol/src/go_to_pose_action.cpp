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

    void odom_topic_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid, 
        std::shared_ptr<const Interface_GoToPose::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle);
    void execute_action(const std::shared_ptr<GoalHandleGoToPose> goal_handle);
    void execute_action_alternate(const std::shared_ptr<GoalHandleGoToPose> goal_handle);    
    void publish_velocity(double linear, double angular);

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

// Functions to convert degrees to radians and vice versa
double rad2deg(double rad) { return rad * 180.0 / M_PI; }
double deg2rad(double deg) { return deg * M_PI / 180.0; }

// Function to calculate Euclidean distance between two Pose2D objects
double calculate_distance(const Pose2D &p1, const Pose2D &p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Calculate required heading, atan2 returns normalized [-π, π]
double calculate_heading(const Pose2D &desired, const Pose2D &current) {
    return atan2(desired.y - current.y, desired.x - current.x);
}

// Function to normalize angle to [-PI, PI]
double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
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
    desired_pos_.theta = normalize_angle(deg2rad(desired_pos_.theta));
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
    rclcpp::Rate loop_rate(5); // 5 Hz
    auto feedback = std::make_shared<Interface_GoToPose::Feedback>();
    auto result = std::make_shared<Interface_GoToPose::Result>();
    double distance, angle_diff;

    // move robot towards destination
    do {
        distance = calculate_distance(desired_pos_, current_pos_);
        angle_diff = calculate_heading(desired_pos_, current_pos_) - current_pos_.theta;
        if (distance > 0.1) {
            publish_velocity(0.2, normalize_angle(angle_diff)); 
        } else { 
            publish_velocity(0.0, 0.0); 
        }
        // Update feedback with current position
        auto temp_pos = current_pos_;
        temp_pos.theta = rad2deg(temp_pos.theta);
        feedback->current_pos = temp_pos;
        goal_handle->publish_feedback(feedback);
        if (goal_handle->is_canceling()) {
            result->status = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Action canceled");
            return;
        }
        loop_rate.sleep();
    } while (rclcpp::ok() && distance > 0.1);
     publish_velocity(0.0, 0.0); 

    // rotate robot to desired yaw
    do {
        angle_diff = normalize_angle(desired_pos_.theta - current_pos_.theta);
        publish_velocity(0, 2*angle_diff);
        // Update feedback with current position
        auto temp_pos = current_pos_;
        temp_pos.theta = rad2deg(temp_pos.theta);
        feedback->current_pos = temp_pos;
        goal_handle->publish_feedback(feedback);
        if (goal_handle->is_canceling()) {
            result->status = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Action canceled");
            return;
        }
        loop_rate.sleep();
    } while (rclcpp::ok() && fabs(angle_diff) > 0.01);
    publish_velocity(0, 0);

    // Goal is done
    if (rclcpp::ok()) {
        result->status = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Action Completed");
    }
}

void GoToPose::execute_action_alternate(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    rclcpp::Rate loop_rate(5); // 5 Hz
    auto feedback = std::make_shared<Interface_GoToPose::Feedback>();
    auto result = std::make_shared<Interface_GoToPose::Result>();
    double distance = calculate_distance(desired_pos_, current_pos_);
    double angle_diff;

    if (distance > 0.1){
        // rotate robot to desired heading
        auto heading = calculate_heading(desired_pos_, current_pos_);
        do {
            angle_diff = normalize_angle(heading - current_pos_.theta);
            publish_velocity(0, 2*angle_diff);
            // Update feedback with current position
            auto temp_pos = current_pos_;
            temp_pos.theta = rad2deg(temp_pos.theta);
            feedback->current_pos = temp_pos;
            goal_handle->publish_feedback(feedback);
            if (goal_handle->is_canceling()) {
                result->status = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Action canceled");
                return;
            }
            loop_rate.sleep();
        } while (rclcpp::ok() && fabs(angle_diff) > 0.01);
        publish_velocity(0, 0); //stop the robot
    }

    // move robot towards destination
    do {
        distance = calculate_distance(desired_pos_, current_pos_);
        angle_diff = calculate_heading(desired_pos_, current_pos_) - current_pos_.theta;
        if (distance > 0.1) {
            publish_velocity(0.2, normalize_angle(angle_diff)); 
        } else { 
            publish_velocity(0.0, 0.0); 
        }
        // Update feedback with current position
        auto temp_pos = current_pos_;
        temp_pos.theta = rad2deg(temp_pos.theta);
        feedback->current_pos = temp_pos;
        goal_handle->publish_feedback(feedback);
        if (goal_handle->is_canceling()) {
            result->status = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Action canceled");
            return;
        }
        loop_rate.sleep();
    } while (rclcpp::ok() && distance > 0.1);
    publish_velocity(0, 0); //stop the robot

    // rotate robot to desired yaw
    do {
        angle_diff = normalize_angle(desired_pos_.theta - current_pos_.theta);
        publish_velocity(0, 2*angle_diff);
        // Update feedback with current position
        auto temp_pos = current_pos_;
        temp_pos.theta = rad2deg(temp_pos.theta);
        feedback->current_pos = temp_pos;
        goal_handle->publish_feedback(feedback);
        if (goal_handle->is_canceling()) {
            result->status = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Action canceled");
            return;
        }
        loop_rate.sleep();
    } while (rclcpp::ok() && fabs(angle_diff) > 0.01);
    publish_velocity(0, 0); //stop the robot

    // Goal is done
    if (rclcpp::ok()) {
        result->status = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Action Completed");
    }
}

void GoToPose::publish_velocity(double linear, double angular) {
    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_msg.linear.x = linear;
    cmd_msg.angular.z = angular;
    cmd_vel_publisher_->publish(cmd_msg);
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
