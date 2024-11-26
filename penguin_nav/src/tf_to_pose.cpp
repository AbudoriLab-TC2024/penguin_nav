#include <chrono>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::TransformStamped;

class TfToPoseNode : public rclcpp::Node {
public:
  TfToPoseNode() : Node("tf_to_pose") {
    this->declare_parameter<std::string>("child_frame_id", "base_link");
    this->get_parameter("child_frame_id", child_frame_id_);

    pose_pub_ =
        this->create_publisher<PoseWithCovarianceStamped>("pcl_pose", 10);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
        1s, std::bind(&TfToPoseNode::timer_callback, this));
  }

private:
  void timer_callback() {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform("map", child_frame_id_,
                                      tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform map to %s: %s",
                  child_frame_id_.c_str(), ex.what());
      return;
    }
    auto pose = std::make_shared<PoseWithCovarianceStamped>();
    pose->header.stamp = this->now();
    pose->header.frame_id = "map";
    pose->pose.pose.position.x = t.transform.translation.x;
    pose->pose.pose.position.y = t.transform.translation.y;
    pose->pose.pose.position.z = t.transform.translation.z;
    pose->pose.pose.orientation = t.transform.rotation;

    pose_pub_->publish(*pose);
  }

  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string child_frame_id_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfToPoseNode>());
  rclcpp::shutdown();
  return 0;
}
