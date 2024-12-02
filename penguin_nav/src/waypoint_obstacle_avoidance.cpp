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
using geometry_msgs::msg::PoseStamped;
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
  SUBCASE("cannot adjust") {
    auto p = make_pose(5, 5, 0);
    auto result = adjust_lateral(map, p, 0.9, 0.9, 0.1);
    test(result, {p, false});
  }
  SUBCASE("not torelance") {
    auto p = make_pose(5, 5, 0);
    auto result = adjust_lateral(map, p, 0.0, 0.0, 0.1);
    test(result, {p, false});
  }
}

std::vector<PoseStamped> cut_behind(const std::vector<PoseStamped> &pts,
                                    const Point &point) {
  size_t i = 0;
  for (; i < pts.size() - 1; i++) {
    tf2::Vector3 p0, p1, e;
    into(pts[i].pose.position, p0);
    into(pts[i + 1].pose.position, p1);
    into(point, e);

    auto v0 = p1 - p0;
    auto v1 = e - p0;
    auto dot = v0.dot(v1);
    if (dot < 0) {
      break;
    }
  }

  std::vector<PoseStamped> result;
  for (size_t j = i; j < pts.size(); j++) {
    result.push_back(pts[j]);
  }

  return result;
}

TEST_CASE("testing cut_behind") {
  auto make_pose = [](double x, double y) {
    PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    return pose;
  };
  auto make_point = [](double x, double y) {
    Point p;
    p.x = x;
    p.y = y;
    return p;
  };
  auto test = [](const std::vector<PoseStamped> &ans,
                 const std::vector<PoseStamped> &exp) {
    CHECK(ans.size() == exp.size());
    for (size_t i = 0; i < ans.size(); i++) {
      CHECK(ans[i].pose.position.x == exp[i].pose.position.x);
      CHECK(ans[i].pose.position.y == exp[i].pose.position.y);
    }
  };

  SUBCASE("x-straight") {
    std::vector<PoseStamped> pts = {
        make_pose(0, 0), make_pose(1, 0), make_pose(2, 0),
        make_pose(3, 0), make_pose(4, 0), make_pose(5, 0),
    };
    SUBCASE("inside") {
      auto result = cut_behind(pts, make_point(2.5, 0));
      test(result, {pts[3], pts[4], pts[5]});
    }
    SUBCASE("before") {
      auto result = cut_behind(pts, make_point(-1, 0));
      test(result, pts);
    }
    SUBCASE("after") {
      auto result = cut_behind(pts, make_point(6, 0));
      test(result, {pts[5]}); // should have last point
    }
  }

  SUBCASE("y-straight") {
    std::vector<PoseStamped> pts = {
        make_pose(0, 0), make_pose(0, 1), make_pose(0, 2),
        make_pose(0, 3), make_pose(0, 4), make_pose(0, 5),
    };
    SUBCASE("inside") {
      auto result = cut_behind(pts, make_point(0, 2.5));
      test(result, {pts[3], pts[4], pts[5]});
    }
    SUBCASE("before") {
      auto result = cut_behind(pts, make_point(0, -1));
      test(result, pts);
    }
    SUBCASE("after") {
      auto result = cut_behind(pts, make_point(0, 6));
      test(result, {pts[5]}); // should have last point
    }
  }

  SUBCASE("u-turn") {
    std::vector<PoseStamped> pts = {make_pose(0, 0), make_pose(1, 0),
                                    make_pose(2, 1), make_pose(1, 2),
                                    make_pose(0, 2)};
    SUBCASE("inside") {
      auto result = cut_behind(pts, make_point(0.5, 0.4));
      test(result, {pts[1], pts[2], pts[3], pts[4]});
    }
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

  need_adjust_service_ =
      this->create_service<penguin_nav_msgs::srv::NeedAdjust>(
          "need_adjust",
          std::bind(&WaypointObstacleAvoidanceNode::need_adjust_callback, this,
                    std::placeholders::_1, std::placeholders::_2));

  global_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "global_costmap/costmap", 10,
      std::bind(&WaypointObstacleAvoidanceNode::global_costmap_callback, this,
                std::placeholders::_1));

  pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "pcl_pose", 10,
          std::bind(&WaypointObstacleAvoidanceNode::pose_callback, this,
                    std::placeholders::_1));
}

void WaypointObstacleAvoidanceNode::adjust_waypoints_callback(
    const std::shared_ptr<penguin_nav_msgs::srv::AdjustWaypoints::Request>
        request,
    std::shared_ptr<penguin_nav_msgs::srv::AdjustWaypoints::Response>
        response) {
  RCLCPP_DEBUG(this->get_logger(), "Received request to adjust waypoints");
  response->modified = false;
  response->waypoints = request->waypoints;

  if (!global_costmap_) {
    return;
  }

  for (size_t i = 0; i < request->waypoints.size(); i++) {
    const auto &p = request->waypoints[i].pose;
    const auto left_torelance = request->left_torelances[i];
    const auto right_torelance = request->right_torelances[i];

    auto ret = adjust_lateral(*global_costmap_, p, left_torelance,
                              right_torelance, 0.1);
    if (ret.modified) {
      response->waypoints[i].pose = ret.pose;
      response->modified = true;
    }
  }
  if (response->modified && pose_) {
    response->waypoints =
        cut_behind(response->waypoints, pose_->pose.pose.position);
  }
}

void WaypointObstacleAvoidanceNode::need_adjust_callback(
    const std::shared_ptr<penguin_nav_msgs::srv::NeedAdjust::Request> request,
    std::shared_ptr<penguin_nav_msgs::srv::NeedAdjust::Response> response) {
  RCLCPP_DEBUG(this->get_logger(), "Received request to check need adjust");
  response->need_adjust = false;

  if (!global_costmap_) {
    return;
  }

  for (const auto &p : request->waypoints) {
    if (is_occupied(*global_costmap_, p.pose.position)) {
      response->need_adjust = true;
      break;
    }
  }
}

void WaypointObstacleAvoidanceNode::global_costmap_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  global_costmap_ = msg;
}

void WaypointObstacleAvoidanceNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  pose_ = msg;
}

} // namespace penguin_nav
