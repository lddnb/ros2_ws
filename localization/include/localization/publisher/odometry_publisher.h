# pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>

namespace location {
class OdometryPublisher {
    public:
    OdometryPublisher(const rclcpp::Node::SharedPtr& node,
                      std::string topic_name,
                      std::string base_frame_id,
                      std::string child_frame_id,
                      size_t buff_size);
    
    void Publish(Eigen::Matrix4f& transform_matrix);

    private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    nav_msgs::msg::Odometry odometry_;
};

}
