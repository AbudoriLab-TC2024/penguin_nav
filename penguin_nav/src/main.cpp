#include "waypoint_obstacle_avoidance.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<penguin_nav::WaypointObstacleAvoidanceNode>());
  rclcpp::shutdown();

  return 0;
}
