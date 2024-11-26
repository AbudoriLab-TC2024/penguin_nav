#include "waypoint_obstacle_avoidance.hpp"
#include "doctest.h"

namespace penguin_nav {

WaypointObstacleAvoidanceNode::WaypointObstacleAvoidanceNode()
    : rclcpp::Node("waypoint_obstacle_avoidance") {
  adjust_waypoints_service_ =
      this->create_service<penguin_nav_msgs::srv::AdjustWaypoints>(
          "adjust_waypoints",
          std::bind(&WaypointObstacleAvoidanceNode::adjust_waypoints_callback,
                    this, std::placeholders::_1, std::placeholders::_2));
  global_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "global_costmap/costmap", 10,
      std::bind(&WaypointObstacleAvoidanceNode::global_costmap_callback, this,
                std::placeholders::_1));
}

void WaypointObstacleAvoidanceNode::adjust_waypoints_callback(
    const std::shared_ptr<penguin_nav_msgs::srv::AdjustWaypoints::Request>
        request,
    std::shared_ptr<penguin_nav_msgs::srv::AdjustWaypoints::Response>
        response) {
  RCLCPP_INFO(this->get_logger(), "Received request to adjust waypoints");
  response->modified = true;
}

void WaypointObstacleAvoidanceNode::global_costmap_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "%d %d %lf %lf %lf", msg->info.width,
              msg->info.height, msg->info.resolution,
              msg->info.origin.position.x, msg->info.origin.position.y);
  global_costmap_ = msg;
}

} // namespace penguin_nav
