#pragma once
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <penguin_nav_msgs/srv/adjust_waypoints.hpp>
#include <rclcpp/rclcpp.hpp>

namespace penguin_nav {
class WaypointObstacleAvoidanceNode : public rclcpp::Node {
public:
  WaypointObstacleAvoidanceNode();

private:
  void adjust_waypoints_callback(
      const std::shared_ptr<penguin_nav_msgs::srv::AdjustWaypoints::Request>
          request,
      std::shared_ptr<penguin_nav_msgs::srv::AdjustWaypoints::Response>
          response);

  void
  global_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  void pose_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  rclcpp::Service<penguin_nav_msgs::srv::AdjustWaypoints>::SharedPtr
      adjust_waypoints_service_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
      global_costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_sub_;

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr global_costmap_;
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_;
};

} // namespace penguin_nav
