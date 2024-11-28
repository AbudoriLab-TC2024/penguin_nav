#include "waypoint_obstacle_avoidance.hpp"
#include "doctest.h"
#include "util.hpp"
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace penguin_nav {

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::OccupancyGrid;

namespace {
template <typename P> bool is_occupied(const OccupancyGrid &map, const P &p) {
  auto width = map.info.width;
  auto height = map.info.height;
  auto resolution = map.info.resolution;
  auto ox = map.info.origin.position.x;
  auto oy = map.info.origin.position.y;

  auto x = (p.x - ox) / resolution;
  auto y = (p.y - oy) / resolution;

  if (x < 0 || x >= width || y < 0 || y >= height) {
    return false;
  }
  auto ix = std::round(x);
  auto iy = std::round(y);

  auto index = iy * width + ix;
  auto data = map.data[index];
  return data > 0;
}

TEST_CASE("testing is_occupied") {
  // Create 5mx10m costmap with resolution 0.1, origin at (-1, -2)
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = 50;
  map.info.height = 100;
  map.info.resolution = 0.1;
  map.info.origin.position.x = -1;
  map.info.origin.position.y = -2;

  // Set all cells to 0
  map.data.resize(map.info.width * map.info.height, 0);

  // Fill x=1-2, y=3-4 with obstacles
  for (size_t y = 30; y < 40; y++) {
    for (size_t x = 10; x < 20; x++) {
      auto index = y * map.info.width + x;
      map.data[index] = 100;
    }
  }

  auto p = [](double x, double y) {
    geometry_msgs::msg::Point pt;
    pt.x = x;
    pt.y = y;
    return pt;
  };

  // outside costmap
  CHECK(is_occupied(map, p(-2, -3)) == false);
  CHECK(is_occupied(map, p(5, 10)) == false);

  // inside costmap
  CHECK(is_occupied(map, p(0, 0)) == false);

  CHECK(is_occupied(map, p(0, 1)) == true);
  CHECK(is_occupied(map, p(0.9, 1.9)) == true);

  // check rounding
  CHECK(is_occupied(map, p(0.96, 1.96)) == false);
}

struct AdjustedPose {
  geometry_msgs::msg::Pose pose;
  bool modified;
};

AdjustedPose adjust_lateral(const OccupancyGrid &map, const Pose &pose,
                            double left_torelance, double right_torelance,
                            double step) {
  // Adjust laterally if the pose is occupied.
  // 'modified' will be true if the pose is adjusted. It will be false even if
  // the pose is occupied but could not be adjusted.
  AdjustedPose result = {pose, false};

  if (!is_occupied(map, pose.position)) {
    return result;
  }

  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z,
                    pose.orientation.w);
  auto left = tf2::quatRotate(q, tf2::Vector3(0, 1, 0));
  tf2::Vector3 origin;
  into(pose.position, origin);

  for (auto delta = step; delta <= left_torelance || delta <= right_torelance;
       delta += step) {
    if (delta <= left_torelance) {
      Point p;
      into(origin + left * delta, p);

      if (!is_occupied(map, p)) {
        result.modified = true;
        result.pose.position = p;
        break;
      }
    }
    if (delta <= right_torelance) {
      Point p;
      into(origin - left * delta, p);
      if (!is_occupied(map, p)) {
        result.modified = true;
        result.pose.position = p;
        break;
      }
    }
  }

  return result;
}

TEST_CASE("testing adjust_lateral") {
  // Create 10mx10m costmapt with resolution 0.1, origin at (0, 0)
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = 100;
  map.info.height = 100;
  map.info.resolution = 0.1;

  // Set all cells to 0
  map.data.resize(map.info.width * map.info.height, 0);

  // Set obstacles at x=4-6, y=4-6
  for (size_t y = 40; y < 60; y++) {
    for (size_t x = 40; x < 60; x++) {
      auto index = y * map.info.width + x;
      map.data[index] = 100;
    }
  }

  auto make_pose = [](double x, double y, double yaw) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    auto q = tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw);
    into(q, pose.orientation);

    return pose;
  };

  auto test = [](const AdjustedPose &ans, const AdjustedPose &exp) {
    CHECK(ans.modified == exp.modified);
    CHECK(ans.pose.position.x == doctest::Approx(exp.pose.position.x));
    CHECK(ans.pose.position.y == doctest::Approx(exp.pose.position.y));
    CHECK(ans.pose.orientation == exp.pose.orientation);
  };

  SUBCASE("no adjust") {
    auto p = make_pose(5, 3, 0);
    auto result = adjust_lateral(map, p, 1, 1, 0.1);
    test(result, {p, false});
  }
  SUBCASE("adjust right") {
    auto p = make_pose(5, 4.5, 0);
    auto result = adjust_lateral(map, p, 1, 1, 0.1);
    test(result, {make_pose(5, 3.9, 0), true});
  }
  SUBCASE("adjust left") {
    auto p = make_pose(5, 5.5, 0);
    auto result = adjust_lateral(map, p, 1, 1, 0.1);
    test(result, {make_pose(5, 6.0, 0), true});
  }
  SUBCASE("adjust right with rotation") {
    auto p = make_pose(4.5, 5, M_PI_2);
    auto result = adjust_lateral(map, p, 1, 1, 0.1);
    test(result, {make_pose(3.9, 5, M_PI_2), true});
  }
  SUBCASE("adjust left with rotation") {
    auto p = make_pose(5.5, 5, M_PI_2);
    auto result = adjust_lateral(map, p, 1, 1, 0.1);
    test(result, {make_pose(6.0, 5, M_PI_2), true});
  }
}

} // namespace

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
