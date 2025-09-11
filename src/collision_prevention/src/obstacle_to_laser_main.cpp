/****************************************************************************
 * Copyright (c) 2024 Collision Prevention Node.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include "collision_prevention/obstacle_to_laser_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleToLaserNode>();
  
  RCLCPP_INFO(node->get_logger(), "Starting Obstacle to Laser Node for ObstacleDistance to LaserScan conversion");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}