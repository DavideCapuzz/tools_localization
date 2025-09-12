// Copyright 2019 coderkarl. Subject to the BSD license.

#include "tools_localization/localization_node.hpp"
#include "rclcpp/rclcpp.hpp"
/**********************************************************************
 * Obstacle Avoidance using a nav_msgs/OccupacyGrid and A* path planning
 *
 * Subscribe to /scan and Publish OccupacyGrid /costmap, Publish Path /path
 *
 **********************************************************************/

using std::placeholders::_1;

#ifndef TESTING_EXCLUDE_MAIN
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LocalizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
