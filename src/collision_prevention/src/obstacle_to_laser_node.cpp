/****************************************************************************
 * Copyright (c) 2024 Collision Prevention Node.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "collision_prevention/obstacle_to_laser_node.hpp"

ObstacleToLaserNode::ObstacleToLaserNode() : Node("obstacle_to_laser_node")
{
  // Declare parameters
  this->declare_parameter("input_topic", "/px4_0/fmu/out/obstacle_distance_fused");
  this->declare_parameter("output_topic", "/px4_debug_fused_obstacle_distance_laser_scan");
  this->declare_parameter("frame_id", "base_link");
  this->declare_parameter("debug_output", true);

  // Get parameters
  input_topic_ = this->get_parameter("input_topic").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  debug_output_ = this->get_parameter("debug_output").as_bool();

  // Set up QoS profiles - CRITICAL for PX4 compatibility
  // Subscriber QoS: Use BestEffort for PX4 fmu/out topics (they publish with BestEffort)
  auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .durability(rclcpp::DurabilityPolicy::Volatile);

  // Publisher QoS: Use sensor_data profile for LaserScan (ROS2 standard)
  auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5))
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .durability(rclcpp::DurabilityPolicy::Volatile);

  // Create subscriber and publisher with appropriate QoS
  obstacle_distance_sub_ = this->create_subscription<px4_msgs::msg::ObstacleDistance>(
    input_topic_, px4_qos,
    std::bind(&ObstacleToLaserNode::obstacleDistanceCallback, this, std::placeholders::_1));

  laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    output_topic_, sensor_qos);

  // Initialize counters for debugging
  obstacle_distance_count_ = 0;
  laser_scan_count_ = 0;
  last_publish_time_ = this->get_clock()->now();

  RCLCPP_INFO(this->get_logger(), "=== Obstacle to Laser Scan Node Started ===");
  RCLCPP_INFO(this->get_logger(), "Input topic: %s (QoS: BestEffort)", input_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Output topic: %s (QoS: BestEffort)", output_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "Debug output: %s", debug_output_ ? "ENABLED" : "DISABLED");
  RCLCPP_INFO(this->get_logger(), "Waiting for ObstacleDistance messages on %s...", input_topic_.c_str());
}

void ObstacleToLaserNode::obstacleDistanceCallback(const px4_msgs::msg::ObstacleDistance::SharedPtr msg)
{
  obstacle_distance_count_++;
  
  // First message debug info
  if (obstacle_distance_count_ == 1) {
    RCLCPP_INFO(this->get_logger(), "FIRST ObstacleDistance message received");
    RCLCPP_INFO(this->get_logger(), "  Range: [%d, %d] cm", msg->min_distance, msg->max_distance);
    RCLCPP_INFO(this->get_logger(), "  Frame: %d, Sensor type: %d", msg->frame, msg->sensor_type);
    RCLCPP_INFO(this->get_logger(), "  Angle offset: %.2f deg, Increment: %.2f deg", 
                msg->angle_offset, msg->increment);
  }

  // Convert PX4 ObstacleDistance to LaserScan
  sensor_msgs::msg::LaserScan scan_msg;
  
  // Header
  scan_msg.header.stamp = this->get_clock()->now();
  scan_msg.header.frame_id = frame_id_;
  
  // Convert distances from cm to meters
  scan_msg.range_min = static_cast<float>(msg->min_distance / 100.0);
  scan_msg.range_max = static_cast<float>(msg->max_distance / 100.0);
  
  // Use default values if not set properly
  if (scan_msg.range_min <= 0) {
    scan_msg.range_min = DEFAULT_RANGE_MIN;
  }
  if (scan_msg.range_max <= scan_msg.range_min) {
    scan_msg.range_max = DEFAULT_RANGE_MAX;
  }
  
  // Calculate angle parameters
  // PX4 uses angle_offset and increment in degrees
  float angle_offset_rad = msg->angle_offset * M_PI / 180.0f;
  float increment_rad = msg->increment * M_PI / 180.0f;
  
  // Count valid distance measurements
  int valid_distances = 0;
  for (size_t i = 0; i < msg->distances.size(); i++) {
    if (msg->distances[i] != UINT16_MAX && msg->distances[i] > 0) {
      valid_distances++;
    }
  }
  
  if (valid_distances == 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "No valid distance measurements in ObstacleDistance message");
    return;
  }
  
  // Set up laser scan parameters
  // Since we reverse the array, we need to flip the angles too
  float original_angle_max = angle_offset_rad + (valid_distances - 1) * increment_rad;
  scan_msg.angle_min = -original_angle_max;  // Flip and negate
  scan_msg.angle_max = -angle_offset_rad;    // Flip and negate  
  scan_msg.angle_increment = increment_rad;
  scan_msg.time_increment = 0.0;
  scan_msg.scan_time = 0.1; // Assume 10Hz scan rate
  
  // Convert distance measurements
  scan_msg.ranges.clear();
  scan_msg.intensities.clear();
  
  // Create temporary vector to store distances
  std::vector<float> temp_ranges;
  
  for (size_t i = 0; i < msg->distances.size() && i < PX4_OBSTACLE_SECTORS; i++) {
    float distance_m;
    
    if (msg->distances[i] == UINT16_MAX) {
      // Unknown distance - set to max range
      distance_m = scan_msg.range_max;
    } else if (msg->distances[i] == 0) {
      // Obstacle right at sensor
      distance_m = scan_msg.range_min;
    } else {
      // Convert from cm to meters
      distance_m = static_cast<float>(msg->distances[i] / 100.0);
      
      // Clamp to valid range
      if (distance_m < scan_msg.range_min) {
        distance_m = scan_msg.range_min;
      } else if (distance_m > scan_msg.range_max) {
        distance_m = scan_msg.range_max;
      }
    }
    
    temp_ranges.push_back(distance_m);
  }
  
  // Reverse the array because PX4 ObstacleDistance is in FRD frame 
  // but LaserScan should be in FLU frame (reverse the original transformation)
  for (auto it = temp_ranges.rbegin(); it != temp_ranges.rend(); ++it) {
    scan_msg.ranges.push_back(*it);
    scan_msg.intensities.push_back(100.0); // Default intensity value
  }

  laser_scan_pub_->publish(scan_msg);
  laser_scan_count_++;

  // First publish debug info
  if (laser_scan_count_ == 1) {
    RCLCPP_INFO(this->get_logger(), "FIRST LaserScan message published to %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Range: [%.3f, %.3f] m", scan_msg.range_min, scan_msg.range_max);
    RCLCPP_INFO(this->get_logger(), "  Angle: [%.2f, %.2f] deg", 
                scan_msg.angle_min * 180.0 / M_PI, scan_msg.angle_max * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "  Samples: %zu, Increment: %.2f deg", 
                scan_msg.ranges.size(), scan_msg.angle_increment * 180.0 / M_PI);
  }

  // Periodic detailed debug info
  if (debug_output_ && (laser_scan_count_ % 50 == 0)) {
    auto now = this->get_clock()->now();
    auto duration = now - last_publish_time_;
    double hz = 50.0 / (duration.seconds());
    last_publish_time_ = now;
    
    RCLCPP_INFO(this->get_logger(), "LaserScan message #%ld published (%.1f Hz)", 
                laser_scan_count_, hz);
    RCLCPP_INFO(this->get_logger(), "  Valid distances in ObstacleDistance: %d/%zu",
                valid_distances, msg->distances.size());
    
    // Show first few ranges for debugging
    std::string ranges_str = "  First 8 ranges (m): ";
    for (size_t i = 0; i < 8 && i < scan_msg.ranges.size(); i++) {
      ranges_str += std::to_string(scan_msg.ranges[i]) + " ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", ranges_str.c_str());
  }

  RCLCPP_DEBUG(this->get_logger(), 
    "Published LaserScan with %zu ranges, min_range: %.3f m, max_range: %.3f m",
    scan_msg.ranges.size(), scan_msg.range_min, scan_msg.range_max);
}