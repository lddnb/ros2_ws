#include "publisher/subscriber_function.hpp"

using std::placeholders::_1;

Sub::Sub():Node("Subscription") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&Sub::topic_callback, this, _1));
}

void Sub::topic_callback(const std_msgs::msg::String & msg) {
    RCLCPP_INFO(this->get_logger(), "I hear: %s", msg.data.c_str());
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sub>());
    rclcpp::shutdown();
    return 0;
}