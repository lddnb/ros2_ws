#include "learning_tf2_cpp/broadcaster.hpp"

Broadcaster::Broadcaster(std::string name):Node(name) {
    br = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void Broadcaster::HandleMsg(const geometry_msgs::msg::TransformStamped & transfrom) {
    br->sendTransform(transfrom);
}