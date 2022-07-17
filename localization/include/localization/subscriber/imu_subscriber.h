#pragma once

#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <sensor_msgs/msg/imu.hpp>

#include "sensor_data/imu_data.h"

namespace location {
class IMUSubscriber {
    public:
    IMUSubscriber(const rclcpp::Node::SharedPtr& node, std::string topic_name, size_t buff_size);

    void ParseData(std::deque<ImuData>& deque_imu_data);


    private:
    void msg_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg_ptr);

    private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;

    std::deque<ImuData> new_imu_data_;
};

}