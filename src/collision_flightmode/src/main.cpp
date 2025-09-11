/****************************************************************************
 * Copyright (c) 2024 Collision Autonomous Flight Mode.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "rclcpp/rclcpp.hpp"

#include "collision_flightmode/collision_autonomous_mode.hpp"
#include <px4_ros2/components/node_with_mode.hpp>

using MyNodeWithMode = px4_ros2::NodeWithMode<CollisionAutonomousMode>;

static const std::string kNodeName = "collision_autonomous_mode_node";
static const bool kEnableDebugOutput = true;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Collision Aware Autonomous Flight Mode");
  
  rclcpp::spin(std::make_shared<MyNodeWithMode>(kNodeName, kEnableDebugOutput));
  
  rclcpp::shutdown();
  return 0;
}