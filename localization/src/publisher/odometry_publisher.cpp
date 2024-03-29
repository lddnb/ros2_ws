#include "publisher/odometry_publisher.h"

namespace location {
OdometryPublisher::OdometryPublisher(const rclcpp::Node::SharedPtr& node,
                                     std::string topic_name,
                                     std::string base_frame_id,
                                     std::string child_frame_id,
                                     size_t buff_size) {
    node_ = node;
    publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(topic_name, buff_size);
    odometry_.header.frame_id = base_frame_id;
    odometry_.child_frame_id = child_frame_id;
}

void OdometryPublisher::Publish(Eigen::Matrix4f& transform_matrix) {
    odometry_.header.stamp = node_->get_clock()->now();

    //set the position
    odometry_.pose.pose.position.x = transform_matrix(0,3);
    odometry_.pose.pose.position.y = transform_matrix(1,3);
    odometry_.pose.pose.position.z = transform_matrix(2,3);

    Eigen::Quaternionf q;
    q = transform_matrix.block<3,3>(0,0);
    odometry_.pose.pose.orientation.x = q.x();
    odometry_.pose.pose.orientation.y = q.y();
    odometry_.pose.pose.orientation.z = q.z();
    odometry_.pose.pose.orientation.w = q.w();

    publisher_->publish(odometry_);
}

}