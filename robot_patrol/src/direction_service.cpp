#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node {
    public:
    DirectionService() : Node("direction_service_server"){
        getDirectionService_ = create_service<robot_patrol::srv::GetDirection>("direction_service", std::bind(&DirectionService::direction_service_callback, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Service Server Ready");
    }
    private:
    rclcpp::Service<robot_patrol::srv::GetDirection>::SharedPtr getDirectionService_;
    void direction_service_callback(
        const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
        const std::shared_ptr<robot_patrol::srv::GetDirection::Response> response);
};

void DirectionService::direction_service_callback(
        const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
        const std::shared_ptr<robot_patrol::srv::GetDirection::Response> response){
            RCLCPP_INFO(this->get_logger(), "Service Request");
            auto data = request->laser_data;
            auto ranges = data.ranges;

            auto convert_angle = [&](double angle){
                if (angle >= data.angle_min && angle <= data.angle_max) {
                    return static_cast<int>((angle - data.angle_min) / data.angle_increment);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Input angle out of range");
                    return -1;
                }
            };

            int index_right_start = convert_angle(-M_PI_2), index_right_end = convert_angle(-M_PI/6);
            int index_front_start = index_right_end, index_front_end = convert_angle(M_PI/6);
            int index_left_start = index_front_end, index_left_end = convert_angle(M_PI_2);

            double total_dist_sec_right =0.0, total_dist_sec_front=0.0, total_dist_sec_left =0.0;

            for (int i = index_right_start; i < index_right_end; i++){
                if(std::isfinite(ranges[i])){
                    total_dist_sec_right+= ranges[i];
                }
            }
            for (int i = index_front_start; i < index_front_end; i++){
                if(std::isfinite(ranges[i])){
                    total_dist_sec_front+= ranges[i];
                }
            }
            for (int i = index_left_start; i < index_left_end; i++){
                if(std::isfinite(ranges[i])){
                    total_dist_sec_left+= ranges[i];
                }
            }
            if (total_dist_sec_front >= total_dist_sec_right && total_dist_sec_front >= total_dist_sec_left){
                response->direction = "forward";
            } else if (total_dist_sec_right >= total_dist_sec_front  && total_dist_sec_right >= total_dist_sec_left){
                response->direction = "right";
            } else {
                response->direction = "left";
            }
            RCLCPP_INFO(this->get_logger(), "Service Response: %s", response->direction.c_str());      
        }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}
