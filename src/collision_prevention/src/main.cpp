/****************************************************************************
 * Copyright (c) 2024 Collision Prevention Node.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include "collision_prevention/collision_prevention_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CollisionPreventionNode>();
  
  RCLCPP_INFO(node->get_logger(), "Starting Collision Prevention Node for LaserScan to ObstacleDistance conversion");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}