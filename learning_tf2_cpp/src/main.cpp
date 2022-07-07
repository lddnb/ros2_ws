#include <learning_tf2_cpp/listener.hpp>
#include <learning_tf2_cpp/broadcaster.hpp>
#include <turtlesim/msg/pose.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <turtlesim/srv/spawn.hpp>
#include <string>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("tf2_learning");

    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);

    auto static_br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

    bool service_finished = false;

    //调用服务产生第二只乌龟
    auto spawner_ = node->create_client<turtlesim::srv::Spawn>("spawn");

    auto lr = std::make_shared<Listener>("turtle2", "turtle1");
    geometry_msgs::msg::TransformStamped reslut;

    geometry_msgs::msg::TransformStamped transform;

    transform.header.frame_id = "turtle1";
    transform.header.stamp = node->get_clock()->now();
    transform.child_frame_id = "carrot1";
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 2.0;
    transform.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    static_br->sendTransform(transform);

    rclcpp::WallRate rate(100ms);

    while (rclcpp::ok()) {
        if (!service_finished) {
            if (spawner_->service_is_ready())
            {
                auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
                request->x = 1;
                request->y = 1;
                request->theta = 0;
                request->name = "turtle2";

                auto response_receive_callback = [&](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
                {
                    auto result = future.get();
                    if (strcmp(result->name.c_str(), "turtle2") == 0)
                    {
                        RCLCPP_INFO(node->get_logger(), "Service is sunccess!!!");
                        service_finished = true;
                    }
                    else
                    {
                        RCLCPP_ERROR(node->get_logger(), "Service callback result mismatch");
                    }
                };
                spawner_->async_send_request(request, response_receive_callback);
            }
            else
            {
                RCLCPP_INFO(node->get_logger(), "Service is not ready");
            }
        }
        else {
            lr->Lookup(reslut);
            //RCLCPP_INFO(node->get_logger(), "result = %f, %f", reslut.transform.translation.x, reslut.transform.translation.y);
            geometry_msgs::msg::Twist vel_msg;
            vel_msg.angular.z = atan2(reslut.transform.translation.y, reslut.transform.translation.x);
            //RCLCPP_INFO(node->get_logger(), "angular velosity = %f", vel_msg.angular.z);
            vel_msg.linear.x = sqrt(pow(reslut.transform.translation.x, 2) + pow(reslut.transform.translation.y, 2));
            //RCLCPP_INFO(node->get_logger(), "linear velosity = %f", vel_msg.linear.x);
            pub->publish(vel_msg);
        }
        
        rclcpp::spin_some(node);
        rate.sleep();
    }

    return 0;
}