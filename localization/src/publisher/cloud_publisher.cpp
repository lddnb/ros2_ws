#include "localization/publisher/cloud_publisher.h"
#include <glog/logging.h>

namespace location {
CloudPublisher::CloudPublisher(const rclcpp::Node::SharedPtr& node, std::string topic_name, long long buff_size, std::string frame_id)
    :frame_id_(frame_id) {
    node_ = node;
    publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, buff_size);
}

void CloudPublisher::Publish(CloudData::CLOUD cloud_input) {
    sensor_msgs::msg::PointCloud2 cloud_ptr_output;
    pcl::toROSMsg(cloud_input, cloud_ptr_output);
    cloud_ptr_output.header.stamp = node_->get_clock()->now();
    cloud_ptr_output.header.frame_id = frame_id_;
    publisher_->publish(cloud_ptr_output);
    // LOG(INFO) << "frame_id = " << cloud_ptr_output->header.frame_id << std::endl
    //           << "height = " << cloud_ptr_output->height << std::endl
    //           << "width = " << cloud_ptr_output->width << std::endl;
}

}