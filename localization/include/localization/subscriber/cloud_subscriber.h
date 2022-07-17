#pragma once

#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "sensor_data/cloud_data.h"

namespace location {
class CloudSubscriber {
    public:
    CloudSubscriber(const rclcpp::Node::SharedPtr& node, std::string topic_name, size_t buff_size);

    void ParseData(std::deque<CloudData>& deque_cloud_data);

    private:
    void msg_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);

    private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    std::deque<CloudData> new_cloud_data_;
};
}