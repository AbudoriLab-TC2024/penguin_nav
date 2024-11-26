#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::TransformStamped;

class PoseToTfNode : public rclcpp::Node {
public:
  PoseToTfNode() : Node("pose_to_tf") {
    this->declare_parameter<std::string>("child_frame_id", "base_link");
    this->get_parameter("child_frame_id", child_frame_id_);

    pose_sub_ = this->create_subscription<PoseWithCovarianceStamped>(
        "pose", 10,
        std::bind(&PoseToTfNode::pose_callback, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  void pose_callback(const PoseWithCovarianceStamped::SharedPtr pose) {
    TransformStamped tf;

    tf.header.stamp = pose->header.stamp;
    tf.header.frame_id = pose->header.frame_id;
    tf.child_frame_id = child_frame_id_;

    tf.transform.translation.x = pose->pose.pose.position.x;
    tf.transform.translation.y = pose->pose.pose.position.y;
    tf.transform.translation.z = pose->pose.pose.position.z;

    tf.transform.rotation = pose->pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf);
  }

  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string child_frame_id_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseToTfNode>());
  rclcpp::shutdown();
  return 0;
}
