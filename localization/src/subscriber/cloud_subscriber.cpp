#include "localization/subscriber/cloud_subscriber.h"

using std::placeholders::_1;

namespace location {
CloudSubscriber::CloudSubscriber(const rclcpp::Node::SharedPtr& node, std::string topic_name, size_t buff_size) {
    node_ = node;
    subscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, buff_size, std::bind(&CloudSubscriber::msg_callback, this, _1));
}

void CloudSubscriber::ParseData(std::deque<CloudData>& deque_cloud_data) {
    if (new_cloud_data_.size() > 0) {
        deque_cloud_data.insert(deque_cloud_data.end(), new_cloud_data_.begin(), new_cloud_data_.end());
        new_cloud_data_.clear();
    }
}

void CloudSubscriber::msg_callback(const sensor_msgs::msg::PointCloud2& cloud_msg) {
    CloudData cloud_data;
    cloud_data.time = cloud_msg.header.stamp.sec;
    pcl::fromROSMsg(cloud_msg, cloud_data.cloud);

    new_cloud_data_.emplace_back(cloud_data);
}

}