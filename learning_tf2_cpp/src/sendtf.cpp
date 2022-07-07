#include <learning_tf2_cpp/broadcaster.hpp>
#include <turtlesim/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <string>

using namespace std::chrono_literals;

std::string turtle_name;

void cb(const turtlesim::msg::Pose & msg) {
    static auto br = std::make_shared<Broadcaster>("turtle_boardcaster");
    
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = br->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = turtle_name;
    
    t.transform.translation.x = msg.x;
    t.transform.translation.y = msg.y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, msg.theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    br->HandleMsg(t);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("sendtf");

    node->declare_parameter<std::string>("turtle_name", "turtle1");
    node->get_parameter("turtle_name", turtle_name);

    auto sub = node->create_subscription<turtlesim::msg::Pose>(turtle_name+"/pose", 10, &cb);

    rclcpp::spin(node);

    return 0;
}