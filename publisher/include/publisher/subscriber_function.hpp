# pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class Sub : public rclcpp::Node {
    public:
    Sub();

    private:
    void topic_callback(const std_msgs::msg::String & msg);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};