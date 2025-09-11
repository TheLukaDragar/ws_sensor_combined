/****************************************************************************
 * Copyright (c) 2024 Collision Prevention Node.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "collision_prevention/collision_prevention_node.hpp"

CollisionPreventionNode::CollisionPreventionNode() : Node("collision_prevention_node")
{
  // Declare parameters
  this->declare_parameter("input_topic", "/ld19/scan");
  this->declare_parameter("output_topic", "/fmu/in/obstacle_distance");
  this->declare_parameter("sector_size_deg", DEFAULT_SECTOR_SIZE_DEG);
  this->declare_parameter("debug_output", true);
  this->declare_parameter("publish_rate_hz", 100);

  // Get parameters
  input_topic_ = this->get_parameter("input_topic").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();
  sector_size_deg_ = this->get_parameter("sector_size_deg").as_int();
  debug_output_ = this->get_parameter("debug_output").as_bool();
  publish_rate_ = this->get_parameter("publish_rate_hz").as_int();

  // Set up QoS profiles - CRITICAL for PX4 compatibility
  // Subscriber QoS: Use sensor_data profile for LaserScan (ROS2 standard)
  auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5))
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .durability(rclcpp::DurabilityPolicy::Volatile);

  // Publisher QoS: Use default for PX4 compatibility (reliable)
  auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::Volatile);

  // Create subscriber and publisher with appropriate QoS
  laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    input_topic_, sensor_qos,
    std::bind(&CollisionPreventionNode::laserScanCallback, this, std::placeholders::_1));

  obstacle_distance_pub_ = this->create_publisher<px4_msgs::msg::ObstacleDistance>(
    output_topic_, px4_qos);

  // Initialize counters for debugging
  laser_scan_count_ = 0;
  obstacle_distance_count_ = 0;
  last_publish_time_ = this->get_clock()->now();

  RCLCPP_INFO(this->get_logger(), "=== Collision Prevention Node Started ===");
  RCLCPP_INFO(this->get_logger(), "Input topic: %s (QoS: BestEffort)", input_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Output topic: %s (QoS: Reliable)", output_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Sector size: %d degrees (%d sectors total)", sector_size_deg_, PX4_OBSTACLE_SECTORS);
  RCLCPP_INFO(this->get_logger(), "Debug output: %s", debug_output_ ? "ENABLED" : "DISABLED");
  RCLCPP_INFO(this->get_logger(), "Waiting for LaserScan messages on %s...", input_topic_.c_str());
}

void CollisionPreventionNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  laser_scan_count_++;
  
  // First message debug info
  if (laser_scan_count_ == 1) {
    RCLCPP_INFO(this->get_logger(), "FIRST LaserScan message received");
    RCLCPP_INFO(this->get_logger(), "  Range: [%.2f, %.2f] m", msg->range_min, msg->range_max);
    RCLCPP_INFO(this->get_logger(), "  Angle: [%.2f, %.2f] deg", 
                msg->angle_min * 180.0 / M_PI, msg->angle_max * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "  Samples: %zu, Increment: %.2f deg", 
                msg->ranges.size(), msg->angle_increment * 180.0 / M_PI);
  }

  // Convert angles from radians to degrees
  double angle_min_deg = msg->angle_min * 180.0 / M_PI;
  double angle_step_deg = msg->angle_increment * 180.0 / M_PI;
  
  // Calculate samples per sector and number of sectors
  int samples_per_sector = std::round(static_cast<double>(sector_size_deg_) / angle_step_deg);
  int number_of_sectors = static_cast<int>(msg->ranges.size()) / samples_per_sector;
  
  if (samples_per_sector <= 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Invalid samples_per_sector: %d. Check laser scan angle_increment (%.4f deg).", 
      samples_per_sector, angle_step_deg);
    return;
  }

  if (debug_output_ && (laser_scan_count_ % 50 == 0)) {
    RCLCPP_INFO(this->get_logger(), "Processing LaserScan #%ld: %d samples -> %d sectors (%d samples/sector)",
                laser_scan_count_, static_cast<int>(msg->ranges.size()), number_of_sectors, samples_per_sector);
  }

  // Create downsampled array
  std::vector<double> ds_array(number_of_sectors, UINT16_MAX);

  // Count valid ranges for debugging
  int valid_ranges = 0, inf_ranges = 0, out_of_range = 0;

  // Downsample -- take average of samples per sector (following PX4 pattern)
  for (int i = 0; i < number_of_sectors; i++) {
    double sum = 0.0;
    int samples_used_in_sector = 0;

    for (int j = 0; j < samples_per_sector; j++) {
      int index = i * samples_per_sector + j;
      if (index >= static_cast<int>(msg->ranges.size())) {
        break;
      }

      double distance = msg->ranges[index];

      // inf values mean no object (following PX4 pattern)
      if (std::isinf(distance) || std::isnan(distance)) {
        inf_ranges++;
        continue;
      }

      // Filter out invalid ranges
      if (distance < msg->range_min || distance > msg->range_max) {
        out_of_range++;
        continue;
      }

      sum += distance;
      samples_used_in_sector++;
      valid_ranges++;
    }

    // If all samples in a sector are inf then it means the sector is clear
    if (samples_used_in_sector == 0) {
      ds_array[i] = msg->range_max;
    } else {
      ds_array[i] = sum / samples_used_in_sector;
    }
  }

  // Create and publish ObstacleDistance message
  px4_msgs::msg::ObstacleDistance report;

  // Initialize all distances to unknown (following PX4 pattern)
  std::fill(report.distances.begin(), report.distances.end(), UINT16_MAX);

  report.timestamp = this->get_clock()->now().nanoseconds() / 1000; // Convert to microseconds
  report.frame = px4_msgs::msg::ObstacleDistance::MAV_FRAME_BODY_FRD;
  report.sensor_type = px4_msgs::msg::ObstacleDistance::MAV_DISTANCE_SENSOR_LASER;
  report.min_distance = static_cast<uint16_t>(msg->range_min * 100.0); // Convert to cm
  report.max_distance = static_cast<uint16_t>(msg->range_max * 100.0); // Convert to cm
  report.angle_offset = static_cast<float>(angle_min_deg);
  report.increment = static_cast<float>(sector_size_deg_);

  // Map samples in FOV into sectors in ObstacleDistance
  int index = 0;
  int sectors_with_data = 0;

  // Iterate in reverse because array is FLU and we need FRD (following PX4 pattern)
  for (auto it = ds_array.rbegin(); it != ds_array.rend() && index < PX4_OBSTACLE_SECTORS; ++it, ++index) {
    uint16_t distance_cm = static_cast<uint16_t>(*it * 100.0);

    if (distance_cm >= report.max_distance) {
      report.distances[index] = report.max_distance + 1;
    } else if (distance_cm < report.min_distance) {
      report.distances[index] = 0; // Obstacle right at sensor
    } else {
      report.distances[index] = distance_cm;
      sectors_with_data++;
    }
  }

  obstacle_distance_pub_->publish(report);
  obstacle_distance_count_++;

  // First publish debug info
  if (obstacle_distance_count_ == 1) {
    RCLCPP_INFO(this->get_logger(), "FIRST ObstacleDistance message published to %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Frame: MAV_FRAME_BODY_FRD, Sensor: LASER");
    RCLCPP_INFO(this->get_logger(), "  Range: [%d, %d] cm, Angle offset: %.1f deg, Increment: %.1f deg", 
                report.min_distance, report.max_distance, report.angle_offset, report.increment);
    RCLCPP_INFO(this->get_logger(), "  Sectors with valid data: %d/%d", sectors_with_data, number_of_sectors);
  }

  // Periodic detailed debug info
  if (debug_output_ && (obstacle_distance_count_ % 100 == 0)) {
    auto now = this->get_clock()->now();
    auto duration = now - last_publish_time_;
    double hz = 100.0 / (duration.seconds());
    last_publish_time_ = now;
    
    RCLCPP_INFO(this->get_logger(), "ObstacleDistance message #%ld published (%.1f Hz, target: %d Hz)", 
                obstacle_distance_count_, hz, publish_rate_);
    RCLCPP_INFO(this->get_logger(), "  Valid ranges: %d, Inf ranges: %d, Out-of-range: %d, Sectors with data: %d/%d",
                valid_ranges, inf_ranges, out_of_range, sectors_with_data, number_of_sectors);
    
    // Show first few sector distances for debugging
    std::string distances_str = "  First 8 sectors (cm): ";
    for (int i = 0; i < 8 && i < number_of_sectors; i++) {
      if (report.distances[i] == UINT16_MAX) {
        distances_str += "UNKNOWN ";
      } else {
        distances_str += std::to_string(report.distances[i]) + " ";
      }
    }
    RCLCPP_INFO(this->get_logger(), "%s", distances_str.c_str());
  }

  RCLCPP_DEBUG(this->get_logger(), 
    "Published ObstacleDistance with %d sectors, min_dist: %d cm, max_dist: %d cm",
    number_of_sectors, report.min_distance, report.max_distance);
}