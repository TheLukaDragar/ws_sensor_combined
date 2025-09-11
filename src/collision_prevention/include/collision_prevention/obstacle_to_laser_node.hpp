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

class ObstacleToLaserNode : public rclcpp::Node
{
public:
  ObstacleToLaserNode();

private:
  void obstacleDistanceCallback(const px4_msgs::msg::ObstacleDistance::SharedPtr msg);
  
  // ROS2 subscribers and publishers
  rclcpp::Subscription<px4_msgs::msg::ObstacleDistance>::SharedPtr obstacle_distance_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;

  // Configuration parameters
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  bool debug_output_;
  
  // Debug counters and timing
  size_t obstacle_distance_count_;
  size_t laser_scan_count_;
  rclcpp::Time last_publish_time_;
  
  // Constants - following PX4 conventions
  static constexpr int PX4_OBSTACLE_SECTORS = 72;  // PX4 expects exactly 72 sectors
  static constexpr double DEFAULT_RANGE_MIN = 0.05; // 5cm minimum range
  static constexpr double DEFAULT_RANGE_MAX = 10.0; // 10m maximum range
};