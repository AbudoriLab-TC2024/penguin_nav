#pragma once
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace penguin_nav {
void into(const geometry_msgs::msg::Point &src, tf2::Vector3 &dst);
void into(const tf2::Vector3 &src, geometry_msgs::msg::Point &dst);
void into(const geometry_msgs::msg::Quaternion &src, tf2::Quaternion &dst);
void into(const tf2::Quaternion &src, geometry_msgs::msg::Quaternion &dst);

} // namespace penguin_nav
