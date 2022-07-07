#include "localization/subscriber/gnss_subscriber.h"

using std::placeholders::_1;

namespace location {
GnssSubscriber::GnssSubscriber(const rclcpp::Node::SharedPtr& node, std::string topic_name, size_t buff_size) {
    node_ = node;
    subscriber_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        topic_name, buff_size, std::bind(&GnssSubscriber::msg_callback, this, _1)
    );
}    

void GnssSubscriber::ParseData(std::deque<GnssData>& deque_gnss_data) {
    if (new_gnss_data_.size() > 0) {
        deque_gnss_data.insert(deque_gnss_data.end(), new_gnss_data_.begin(), new_gnss_data_.end());
        new_gnss_data_.clear();
    }
}

void GnssSubscriber::msg_callback(const sensor_msgs::msg::NavSatFix& nav_sat_fix_ptr) {
    GnssData gnss_data;
    gnss_data.time = nav_sat_fix_ptr.header.stamp.sec;
    gnss_data.latitude = nav_sat_fix_ptr.latitude;
    gnss_data.longitude = nav_sat_fix_ptr.longitude;
    gnss_data.altitude = nav_sat_fix_ptr.altitude;
    gnss_data.status = nav_sat_fix_ptr.status.status;
    gnss_data.service = nav_sat_fix_ptr.status.service;

    new_gnss_data_.emplace_back(gnss_data);
}

}