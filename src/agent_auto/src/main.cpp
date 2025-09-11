/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "rclcpp/rclcpp.hpp"

#include <mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>

using AgentAutoNode = px4_ros2::NodeWithModeExecutor<AgentAutoExecutor, AgentAutoMode>;

static const std::string kNodeName = "agent_auto";
static const bool kEnableDebugOutput = true;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Create node with PX4 namespace support
  auto node = std::make_shared<AgentAutoNode>(kNodeName, kEnableDebugOutput);
  
  // Set PX4 namespace parameter if provided
  node->declare_parameter("px4_namespace", "px4_0");
  std::string px4_namespace = node->get_parameter("px4_namespace").as_string();
  
  RCLCPP_INFO(node->get_logger(), "Agent Auto mode starting with PX4 namespace: %s", px4_namespace.c_str());
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
