#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_data/cloud_data.h"

namespace location {
class CloudPublisher {
    public:
    CloudPublisher(const rclcpp::Node::SharedPtr& node,
                   std::string topic_name,
                   long long buff_size,
                   std::string frame_id);
    
    void Publish(CloudData::CLOUD cloud_input);

    private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::string frame_id_;
};

}



