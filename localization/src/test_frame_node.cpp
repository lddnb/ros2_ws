#include <pcl/common/transforms.h>
#include <glog/logging.h>
#include <memory>

#include "localization/global_definition/global_definition.h"
#include "localization/subscriber/cloud_subscriber.h"
#include "localization/subscriber/gnss_subscriber.h"
#include "localization/subscriber/imu_subscriber.h"
#include "localization/publisher/cloud_publisher.h"
#include "localization/publisher/odometry_publisher.h"
#include "localization/tf_listener/tf_listener.h"

#include <nav_msgs/msg/path.hpp>

#include "common/lua_parameter_dictionary.hpp"
#include "proto/test.pb.h"

using namespace location;

proto::TestInputMessage CreateTestInputOption(common::LuaParameterDictionary* const lua_parameter_dictionary) {
    proto::TestInputMessage option;
    option.test1 = CreateTest2Option(
        lua_parameter_dictionary->GetDictionary("test1").get());
    option.intput_a = lua_parameter_dictionary->GetDouble("intput_a");
    option.intput_b = lua_parameter_dictionary->GetDouble("intput_b");
    option.intput_c = lua_parameter_dictionary->GetDouble("intput_c");
    return option;
}

proto::Test2 CreateTest2Option(common::LuaParameterDictionary* const lua_parameter_dictionary) {
    proto::Test2 option;
    option.a = lua_parameter_dictionary->GetDouble("a");
    option.b = lua_parameter_dictionary->GetDouble("b");
    option.c = lua_parameter_dictionary->GetDouble("c");
    return option;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = std::string(WORK_SPACE_PATH) + "/Log";
    FLAGS_alsologtostderr = 1;

    LOG(INFO) << "glog success" << std::endl;
    //          << "WORK_SPACE_PATH:" << WORK_SPACE_PATH << std::endl;

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_frame_node");

    std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(node, "/kitti/velo/pointcloud", 100000);
    std::shared_ptr<IMUSubscriber> imu_sub_ptr = std::make_shared<IMUSubscriber>(node, "/kitti/oxts/imu", 100000);
    std::shared_ptr<GnssSubscriber> gnss_sub_ptr = std::make_shared<GnssSubscriber>(node, "/kitti/oxts/gps/fix", 100000);
    std::shared_ptr<TFListener> lidar_to_imu_tf_ptr = std::make_shared<TFListener>(node, "imu_link", "velo_link");

    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(node, "current_scan", 100, "map");
    std::shared_ptr<OdometryPublisher> odom_pub_ptr = std::make_shared<OdometryPublisher>(node, "lidar_odom", "map", "lidar", 100);
    auto path_pub_ptr = node->create_publisher<nav_msgs::msg::Path>("tarjectory", 10);

    nav_msgs::msg::Path path;
    path.header.stamp = node->get_clock()->now();
    path.header.frame_id = "map";
    

    std::deque<CloudData> cloud_data_buff;
    std::deque<ImuData> imu_data_buff;
    std::deque<GnssData> gnss_data_buff;
    
    Eigen::Matrix4f lidar_to_imu_tf = Eigen::Matrix4f::Identity();
    bool transform_received = false;
    bool gnss_origin_position_inited = false;

    rclcpp::WallRate rate(100);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        cloud_sub_ptr->ParseData(cloud_data_buff);
        imu_sub_ptr->ParseData(imu_data_buff);
        gnss_sub_ptr->ParseData(gnss_data_buff);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = node->get_clock()->now();
        

        if (!transform_received) {
            if (lidar_to_imu_tf_ptr->LookupData(lidar_to_imu_tf)) {
                transform_received = true;
                //LOG(INFO) << "lidar to imu transform matrix is:" << std::endl << lidar_to_imu_tf;
            }
        }
        else {
            while (cloud_data_buff.size() > 0 && imu_data_buff.size() > 0 && gnss_data_buff.size() > 0) {
                CloudData cloud_data = cloud_data_buff.front();
                ImuData imu_data = imu_data_buff.front();
                GnssData gnss_data = gnss_data_buff.front();

                double d_time = cloud_data.time - imu_data.time;
                if (d_time < -0.05) {
                    cloud_data_buff.pop_front();
                }
                else if (d_time > 0.05) {
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();
                }
                else {
                    cloud_data_buff.pop_front();
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();

                    Eigen::Matrix4f odometry_matrix;

                    if (!gnss_origin_position_inited) {
                        gnss_data.InitOriginPosition();
                        gnss_origin_position_inited = true;
                    }
                    gnss_data.UpdateXYZ();
                    odometry_matrix(0, 3) = gnss_data.local_E;
                    odometry_matrix(1, 3) = gnss_data.local_N;
                    odometry_matrix(2, 3) = gnss_data.local_U;
                    odometry_matrix.block<3, 3>(0, 0) = imu_data.GetOrientationMatrix();
                    odometry_matrix *= lidar_to_imu_tf;

                    pose.pose.position.x = gnss_data.local_E;
                    pose.pose.position.y = gnss_data.local_N;
                    pose.pose.position.z = gnss_data.local_U;
                    Eigen::Quaternionf q(imu_data.GetOrientationMatrix());
                    pose.pose.orientation.x = q.x();
                    pose.pose.orientation.y = q.y();
                    pose.pose.orientation.z = q.z();
                    pose.pose.orientation.w = q.w();

                    path.poses.emplace_back(pose);
                    path_pub_ptr->publish(path);

                    
                    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
                    // Define a translation of 2.5 meters on the x axis.
                    transform_2.translation() << 0.0, 0.0, 0.0;
                    // The same rotation matrix as before; theta radians arround Z axis
                    transform_2.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ()));

                    //odometry_matrix *= transform_2.matrix();

                    
                    CloudData cloud_data_transformed;
                    pcl::transformPointCloud(cloud_data.cloud, cloud_data_transformed.cloud, odometry_matrix);
                    //LOG(INFO) << odometry_matrix;

                    cloud_pub_ptr->Publish(cloud_data_transformed.cloud);
                    odom_pub_ptr->Publish(odometry_matrix);
                }
            }
        }
        rate.sleep();
    }
}