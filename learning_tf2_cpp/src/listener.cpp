#include <learning_tf2_cpp/listener.hpp>

Listener::Listener(std::string base_frame, std::string child_frame, std::string name)
    :Node(name), base_frame_(base_frame), child_frame_(child_frame) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}   

bool Listener::Lookup(geometry_msgs::msg::TransformStamped& transfrom) {
    try {
        rclcpp::Time now = this->get_clock()->now();
        rclcpp::Time past = now - 5s;
        //transfrom = tf_buffer_->lookupTransform(base_frame_, child_frame_, tf2::TimePointZero);
        transfrom = tf_buffer_->lookupTransform(base_frame_, now, child_frame_, past, "world", 100ms);
        //RCLCPP_INFO(this->get_logger(), "Get transform success!");
        return true;
    }   
    catch (tf2::TransformException& ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transfrom from %s to %s: %s",
        base_frame_.c_str(), child_frame_.c_str(), ex.what());
        return false;
    }
}