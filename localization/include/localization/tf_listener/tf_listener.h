#pragma once

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace location {
class TFListener {
    public:
    TFListener(const rclcpp::Node::SharedPtr& node, std::string base_frame_id, std::string child_frame_id);

    bool LookupData(Eigen::Matrix4f& transform_matrix);

    private:
    bool TransformToMatrix(const geometry_msgs::msg::TransformStamped& transform, Eigen::Matrix4f& transform_matrix);

    private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    std::string base_frame_id_;
    std::string child_frame_id_;
};
}