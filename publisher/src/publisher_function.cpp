# include <publisher/publisher_function.hpp>

using namespace std::chrono_literals;

Pub::Pub():Node("Publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&Pub::timer_callback, this));
}

void Pub::timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world" + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publisheing %s", message.data.c_str());
    publisher_->publish(message);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pub>());
    rclcpp::shutdown();
    return 0;
}
