#pragma once 

#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "sensor_data/gnss_data.h"

namespace location {
class GnssSubscriber {
    public:
    GnssSubscriber(const rclcpp::Node::SharedPtr& node, std::string topic_name, size_t buff_size);

    void ParseData(std::deque<GnssData>& deque_gnss_data);

    private:
    void msg_callback(const sensor_msgs::msg::NavSatFix::SharedPtr nav_sat_fix_ptr);

    private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber_;

    std::deque<GnssData> new_gnss_data_;
};

}