/****************************************************************************
 * Copyright (c) 2024 Collision Prevention Node.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <px4_msgs/msg/obstacle_distance.hpp>
#include <vector>
#include <cmath>

class CollisionPreventionNode : public rclcpp::Node
{
public:
  CollisionPreventionNode();

private:
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  
  // ROS2 subscribers and publishers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Publisher<px4_msgs::msg::ObstacleDistance>::SharedPtr obstacle_distance_pub_;

  // Configuration parameters
  std::string input_topic_;
  std::string output_topic_;
  int sector_size_deg_;
  bool debug_output_;
  int publish_rate_;
  
  // Debug counters and timing
  size_t laser_scan_count_;
  size_t obstacle_distance_count_;
  rclcpp::Time last_publish_time_;
  
  // Constants - following PX4 conventions
  static constexpr int PX4_OBSTACLE_SECTORS = 72;  // PX4 expects exactly 72 sectors
  static constexpr int DEFAULT_SECTOR_SIZE_DEG = 5; // PX4 uses 5 degree sectors
};