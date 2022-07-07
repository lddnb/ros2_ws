#pragma once 
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>

class Broadcaster : public rclcpp::Node {
    public:
    Broadcaster(std::string name = "Tf2_Broadcaster");
    void HandleMsg(const geometry_msgs::msg::TransformStamped &transfrom);

    private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> br;

};