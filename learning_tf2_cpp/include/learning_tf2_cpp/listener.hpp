#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class Listener : public rclcpp::Node {
    public:
    Listener(std::string base_frame, std::string child_frame, std::string name = "Tf2_Broadcaster");
    bool Lookup(geometry_msgs::msg::TransformStamped& transfrom);

    private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    std::string base_frame_;
    std::string child_frame_;
};