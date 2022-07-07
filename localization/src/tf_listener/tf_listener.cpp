#include "localization/tf_listener/tf_listener.h"
#include <glog/logging.h>

#include <Eigen/Geometry>

namespace location {
TFListener::TFListener(const rclcpp::Node::SharedPtr& node, std::string base_frame_id, std::string child_frame_id)
    :base_frame_id_(base_frame_id), child_frame_id_(child_frame_id) {
    node_= node;
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

bool TFListener::LookupData(Eigen::Matrix4f& transform_matrix) {
    try {
        geometry_msgs::msg::TransformStamped transform;
        transform = tf_buffer_->lookupTransform(base_frame_id_, child_frame_id_, tf2::TimePointZero);
        TransformToMatrix(transform, transform_matrix);
        return true;
    }
    catch (tf2::TransformException &ex) {
        //LOG(WARNING) << "Look up transform matrix faild";
        return false;
    }
}

bool TFListener::TransformToMatrix(const geometry_msgs::msg::TransformStamped& transform, Eigen::Matrix4f& transform_matrix) {
    Eigen::Translation3f tl_btol(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z
    );

    double roll, pitch, yaw;
    tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w
    );
    tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
    Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());

    transform_matrix = (tl_btol * rot_x_btol * rot_y_btol * rot_z_btol).matrix();

    return true;
}

}