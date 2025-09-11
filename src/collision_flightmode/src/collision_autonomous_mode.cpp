/****************************************************************************
 * Copyright (c) 2024 Collision Autonomous Flight Mode.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "collision_flightmode/collision_autonomous_mode.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <future>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

CollisionAutonomousMode::CollisionAutonomousMode(rclcpp::Node & node)
  : ModeBase(node, kCollisionAwareName)
  , _state(State::Idle)
  , _current_waypoint_index(0)
  , _laser_data_available(false)
  , _vdb_data_available(false)
  , _current_velocity(Eigen::Vector3f::Zero())
  , _last_velocity_command(node.get_clock()->now())
  , _ros_node(node)
{
  // Declare parameters
  _ros_node.declare_parameter("laser_scan_topic", "/ld19_sim/scan");
  _ros_node.declare_parameter("use_vdb_mapping", true);
  _ros_node.declare_parameter("ambient_pointcloud_topic", "/ambient_pointcloud");
  _ros_node.declare_parameter("occupancy_grid_topic", "/vdb_map/occupancy_grid");
  
  // Initialize TF buffer
  _tf_buffer = std::make_shared<tf2_ros::Buffer>(_ros_node.get_clock());
  _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
  
  // Initialize VDB batch raytrace client
  _vdb_raytrace_client = _ros_node.create_client<vdb_mapping_interfaces::srv::BatchRaytrace>("/vdb_mapping/batch_raytrace");
  
  // Create PX4 ROS2 components
  _goto_setpoint = std::make_shared<px4_ros2::GotoSetpointType>(*this);
  _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

  // Create ROS2 subscribers using the node reference
  _waypoint_sub = _ros_node.create_subscription<geometry_msgs::msg::PointStamped>(
    "/collision_flightmode/waypoint", 10,
    std::bind(&CollisionAutonomousMode::waypointCallback, this, std::placeholders::_1));

  _path_sub = _ros_node.create_subscription<nav_msgs::msg::Path>(
    "/collision_flightmode/path", 10,
    std::bind(&CollisionAutonomousMode::pathCallback, this, std::placeholders::_1));

  // Get laser scan topic parameter
  std::string laser_scan_topic = _ros_node.get_parameter("laser_scan_topic").as_string();
  
  _laser_scan_sub = _ros_node.create_subscription<sensor_msgs::msg::LaserScan>(
    laser_scan_topic, 10,
    std::bind(&CollisionAutonomousMode::laserScanCallback, this, std::placeholders::_1));

  // VDB mapping subscriptions
  // Subscribe to VDB mapping data for visualization only
  std::string ambient_pointcloud_topic = _ros_node.get_parameter("ambient_pointcloud_topic").as_string();
  _pointcloud_sub = _ros_node.create_subscription<sensor_msgs::msg::PointCloud2>(
    ambient_pointcloud_topic, 10,
    std::bind(&CollisionAutonomousMode::pointCloudCallback, this, std::placeholders::_1));

  _twist_sub = _ros_node.create_subscription<geometry_msgs::msg::Twist>(
    "/collision_flightmode/cmd_vel", 10,
    std::bind(&CollisionAutonomousMode::twistCallback, this, std::placeholders::_1));

  // Create visualization publishers
  _collision_sphere_pub = _ros_node.create_publisher<sensor_msgs::msg::PointCloud2>(
    "/collision_flightmode/collision_sphere", 10);
  _ray_lines_pub = _ros_node.create_publisher<visualization_msgs::msg::MarkerArray>(
    "/collision_flightmode/ray_lines", 10);
  _hit_voxels_pub = _ros_node.create_publisher<visualization_msgs::msg::MarkerArray>(
    "/collision_flightmode/hit_voxels", 10);

  RCLCPP_INFO(_ros_node.get_logger(), "Collision Aware Autonomous Mode initialized");
  RCLCPP_INFO(_ros_node.get_logger(), "Subscribing to LaserScan data: %s", laser_scan_topic.c_str());
  RCLCPP_INFO(_ros_node.get_logger(), "Subscribing to VDB PointCloud: %s", ambient_pointcloud_topic.c_str());
  RCLCPP_INFO(_ros_node.get_logger(), "Using VDB mapping for 3D collision detection");
  RCLCPP_INFO(_ros_node.get_logger(), "Publishing collision sphere visualization: /collision_flightmode/collision_sphere");
  RCLCPP_INFO(_ros_node.get_logger(), "Waypoint topics: /collision_flightmode/waypoint and /collision_flightmode/path");
  RCLCPP_INFO(_ros_node.get_logger(), "Velocity control topic: /collision_flightmode/cmd_vel");
}

void CollisionAutonomousMode::onActivate()
{
  RCLCPP_INFO(_ros_node.get_logger(), "Collision Aware Autonomous Mode ACTIVATED");
  
  if (_waypoints.empty()) {
    _state = State::Idle;
    RCLCPP_WARN(_ros_node.get_logger(), "No waypoints loaded! Send waypoints to /collision_flightmode/waypoint or /collision_flightmode/path");
  } else {
    _state = State::NavigatingToWaypoint;
    _current_waypoint_index = 0;
    RCLCPP_INFO(_ros_node.get_logger(), "Mission started with %zu waypoints", _waypoints.size());
  }
}

void CollisionAutonomousMode::onDeactivate()
{
  RCLCPP_INFO(_ros_node.get_logger(), "Collision Aware Autonomous Mode DEACTIVATED");
  _state = State::Idle;
}

void CollisionAutonomousMode::updateSetpoint(float dt_s)
{
  (void)dt_s; // Suppress unused parameter warning
  
  // Check for obstacle data timeout
  auto now = _ros_node.get_clock()->now();
  if (_laser_data_available && 
      (now - _last_laser_update).seconds() > kLaserTimeoutSec) {
    RCLCPP_WARN_THROTTLE(_ros_node.get_logger(), *_ros_node.get_clock(), 5000,
      "LaserScan data timeout - proceeding with caution");
    _laser_data_available = false;
  }
  
  // Check VDB data timeout
  if (_vdb_data_available && 
      (now - _last_vdb_update).seconds() > kLaserTimeoutSec) {
    RCLCPP_WARN_THROTTLE(_ros_node.get_logger(), *_ros_node.get_clock(), 5000,
      "VDB mapping data timeout - using laser scan only");
    _vdb_data_available = false;
  }

  // Check for velocity command timeout
  if (_state == State::VelocityControl &&
      (now - _last_velocity_command).seconds() > kVelocityTimeoutSec) {
    RCLCPP_INFO(_ros_node.get_logger(), "Velocity command timeout - switching to idle");
    _state = State::Idle;
    _current_velocity = Eigen::Vector3f::Zero();
  }

  switch (_state) {
    case State::Idle:
      // Hover in place if no mission
      if (_vehicle_local_position->positionXYValid()) {
        Eigen::Vector3f current_pos = _vehicle_local_position->positionNed();
        _goto_setpoint->update(current_pos);
        
        // Keep publishing collision sphere even in idle
        if (_vdb_data_available || _laser_data_available) {
          getObstacleDistanceVDB(Eigen::Vector3f::Zero());
        }
      }
      break;

    case State::NavigatingToWaypoint:
    case State::AvoidingObstacle:
      {
        // Calculate desired navigation setpoint
        Eigen::Vector3f desired_setpoint = calculateNavigationSetpoint();
        
        // Apply collision prevention using best available data
        Eigen::Vector3f safe_setpoint;
        if (_vdb_data_available || _laser_data_available) {
          safe_setpoint = applyCollisionAvoidance(desired_setpoint);
          
          // Update state based on whether we're avoiding obstacles
          if ((safe_setpoint - desired_setpoint).norm() > 0.5f) {
            _state = State::AvoidingObstacle;
          } else {
            _state = State::NavigatingToWaypoint;
          }
          
          // Continuously publish collision sphere visualization
          if (_vehicle_local_position->positionXYValid()) {
            Eigen::Vector3f current_pos = _vehicle_local_position->positionNed();
            getObstacleDistanceVDB(desired_setpoint - current_pos); // This triggers sphere viz
          }
        } else {
          safe_setpoint = desired_setpoint;
          _state = State::NavigatingToWaypoint;
        }

        // Send setpoint to flight controller
        _goto_setpoint->update(safe_setpoint, std::nullopt, kMaxSpeed);
      }
      break;

    case State::VelocityControl:
      {
        if (_vehicle_local_position->positionXYValid()) {
          // Get current position
          Eigen::Vector3f current_pos = _vehicle_local_position->positionNed();
          
          // Calculate desired position by integrating velocity
          Eigen::Vector3f desired_pos = current_pos + _current_velocity;
          
          // Apply collision prevention using best available data
          Eigen::Vector3f safe_setpoint;
          if (_vdb_data_available || _laser_data_available) {
            safe_setpoint = applyCollisionAvoidance(desired_pos);
            
            // Continuously publish collision sphere in velocity mode too
            getObstacleDistanceVDB(_current_velocity);
            
            // Switch to obstacle avoidance if needed
            if ((safe_setpoint - desired_pos).norm() > 0.5f) {
              _state = State::AvoidingObstacle;
            }
          } else {
            safe_setpoint = desired_pos;
          }          // Send setpoint to flight controller with speed
          _goto_setpoint->update(safe_setpoint, std::nullopt, kMaxSpeed);
        }
      }
      break;

    case State::MissionComplete:
      // Hover at final position
      if (!_waypoints.empty() && _vehicle_local_position->positionXYValid()) {
        _goto_setpoint->update(_waypoints.back());
      }
      break;
  }
}

void CollisionAutonomousMode::waypointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  Eigen::Vector3f waypoint(msg->point.x, msg->point.y, msg->point.z);
  _waypoints.clear();
  _waypoints.push_back(waypoint);
  _current_waypoint_index = 0;
  
  RCLCPP_INFO(_ros_node.get_logger(), "New single waypoint received: [%.2f, %.2f, %.2f]",
    waypoint.x(), waypoint.y(), waypoint.z());
  
  if (_state == State::Idle) {
    _state = State::NavigatingToWaypoint;
    RCLCPP_INFO(_ros_node.get_logger(), "Mission activated");
  }
}

void CollisionAutonomousMode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  _waypoints.clear();
  
  for (const auto& pose_stamped : msg->poses) {
    Eigen::Vector3f waypoint(
      pose_stamped.pose.position.x,
      pose_stamped.pose.position.y,
      pose_stamped.pose.position.z
    );
    _waypoints.push_back(waypoint);
  }
  
  _current_waypoint_index = 0;
  
  RCLCPP_INFO(_ros_node.get_logger(), "New path received with %zu waypoints", _waypoints.size());
  
  if (_state == State::Idle && !_waypoints.empty()) {
    _state = State::NavigatingToWaypoint;
    RCLCPP_INFO(_ros_node.get_logger(), "Mission activated");
  }
}

void CollisionAutonomousMode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  _latest_laser_data = *msg;
  _laser_data_available = true;
  _last_laser_update = _ros_node.get_clock()->now();
  
  // Removed spammy debug log
}

void CollisionAutonomousMode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  _latest_pointcloud = *msg;
  _vdb_data_available = true;
  _last_vdb_update = _ros_node.get_clock()->now();
  
  // Removed spammy debug log
}

void CollisionAutonomousMode::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  _latest_occupancy_grid = *msg;
  _vdb_data_available = true;
  _last_vdb_update = _ros_node.get_clock()->now();
  
  // Removed spammy debug log
}

void CollisionAutonomousMode::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // Store velocity command
  _current_velocity = Eigen::Vector3f(
    msg->linear.x,
    msg->linear.y,
    msg->linear.z
  );
  _last_velocity_command = _ros_node.get_clock()->now();

  // Switch to velocity control mode if not already in it
  if (_state != State::VelocityControl && _state != State::AvoidingObstacle) {
    RCLCPP_INFO(_ros_node.get_logger(), "Switching to velocity control mode");
    _state = State::VelocityControl;
  }
}

bool CollisionAutonomousMode::positionReached(const Eigen::Vector3f& target, float tolerance) const
{
  if (!_vehicle_local_position->positionXYValid()) {
    return false;
  }
  
  Eigen::Vector3f current_pos = _vehicle_local_position->positionNed();
  float distance = (current_pos - target).norm();
  return distance < tolerance;
}

Eigen::Vector3f CollisionAutonomousMode::calculateNavigationSetpoint()
{
  if (!_vehicle_local_position->positionXYValid() || _waypoints.empty()) {
    return Eigen::Vector3f(0, 0, 0);
  }
  
  // Get current position
  Eigen::Vector3f current_pos = _vehicle_local_position->positionNed();
  
  // Check if we've reached the current waypoint
  if (positionReached(_waypoints[_current_waypoint_index], kWaypointTolerance)) {
    if (_current_waypoint_index < _waypoints.size() - 1) {
      _current_waypoint_index++;
      RCLCPP_INFO(_ros_node.get_logger(), "Reached waypoint %zu, proceeding to waypoint %zu",
        _current_waypoint_index, _current_waypoint_index + 1);
    } else {
      // Mission complete
      RCLCPP_INFO(_ros_node.get_logger(), "Mission complete - all waypoints reached");
      _state = State::MissionComplete;
      completed(px4_ros2::Result::Success);
      return current_pos; // Hover in place
    }
  }
  
  // Return current target waypoint
  return _waypoints[_current_waypoint_index];
}

Eigen::Vector3f CollisionAutonomousMode::applyCollisionAvoidance(const Eigen::Vector3f& desired_setpoint)
{
  if (!_vehicle_local_position->positionXYValid()) {
    return desired_setpoint;
  }
  
  Eigen::Vector3f current_pos = _vehicle_local_position->positionNed();
  Eigen::Vector3f direction = desired_setpoint - current_pos;
  
  // Check if we're in velocity control mode
  bool is_velocity_mode = (_state == State::VelocityControl);
  float min_distance = is_velocity_mode ? 0.05f : 0.1f;  // Lower threshold for velocity mode
  
  if (direction.norm() < min_distance) {
    // Already at target or no movement, no collision prevention needed
    return desired_setpoint;
  }
  
  // Use VDB collision detection if available, fallback to laser scan
  bool path_blocked = false;
  Eigen::Vector3f safe_direction = Eigen::Vector3f::Zero();
  
  if (_vdb_data_available) {
    // Use 3D VDB collision detection
    path_blocked = isPathBlockedVDB(current_pos, desired_setpoint);
    if (path_blocked) {
      safe_direction = findSafeDirectionVDB(direction);
      RCLCPP_DEBUG(_ros_node.get_logger(), "Using VDB collision avoidance");
    }
  } else if (_laser_data_available) {
    // Fallback to 2D laser scan detection
    path_blocked = isPathBlocked(direction);
    if (path_blocked) {
      safe_direction = findSafeDirection(direction);
      RCLCPP_DEBUG(_ros_node.get_logger(), "Using LaserScan collision avoidance");
    }
  }
  
  if (path_blocked) {
    if (safe_direction.norm() > min_distance) {
      // Safe direction found
      Eigen::Vector3f safe_setpoint;
      
      if (is_velocity_mode) {
        // For velocity mode, scale the safe direction to match original velocity magnitude
        float original_speed = direction.norm();
        safe_direction.normalize();
        safe_direction *= original_speed;
        safe_setpoint = current_pos + safe_direction;
      } else {
        // For position mode, use the safe direction as is
        safe_setpoint = current_pos + safe_direction;
      }
      
      RCLCPP_DEBUG(_ros_node.get_logger(), "Obstacle detected, redirecting");
      return safe_setpoint;
    } else {
      // No safe direction found, stop
      RCLCPP_WARN_THROTTLE(_ros_node.get_logger(), *_ros_node.get_clock(), 1000,
        "No safe direction found - stopping");
      return current_pos;
    }
  }
  
  // Path is clear, use desired setpoint
  return desired_setpoint;
}

bool CollisionAutonomousMode::isPathBlocked(const Eigen::Vector3f& direction) const
{
  Eigen::Vector2f horizontal_dir(direction.x(), direction.y());
  float speed = horizontal_dir.norm();
  
  // Use a lower threshold for velocity mode
  float min_speed = (_state == State::VelocityControl) ? 0.05f : 0.1f;
  if (speed < min_speed) {
    return false;
  }
  
  float angle_deg = std::atan2(horizontal_dir.y(), horizontal_dir.x()) * 180.0f / M_PI;
  float obstacle_distance = getObstacleDistance(angle_deg);
  
  // For velocity mode, scale the collision threshold based on speed
  float threshold = kCollisionThreshold;
  if (_state == State::VelocityControl) {
    // Increase threshold with speed (up to 1.5x at max speed)
    threshold *= (1.0f + 0.5f * std::min(speed / kMaxSpeed, 1.0f));
  }
  
  return obstacle_distance < threshold;
}

Eigen::Vector3f CollisionAutonomousMode::findSafeDirection(const Eigen::Vector3f& desired_direction)
{
  Eigen::Vector2f horizontal_desired(desired_direction.x(), desired_direction.y());
  float speed = horizontal_desired.norm();
  
  // Use a lower threshold for velocity mode
  float min_speed = (_state == State::VelocityControl) ? 0.05f : 0.1f;
  if (speed < min_speed) {
    return Eigen::Vector3f(0, 0, 0);
  }
  
  float desired_angle_deg = std::atan2(horizontal_desired.y(), horizontal_desired.x()) * 180.0f / M_PI;
  
  // Adjust guidance angle based on speed in velocity mode
  float guidance_angle = kGuidanceAngleDeg;
  if (_state == State::VelocityControl) {
    // Increase guidance angle at higher speeds (up to 1.5x at max speed)
    guidance_angle *= (1.0f + 0.5f * std::min(speed / kMaxSpeed, 1.0f));
  }
  
  // Try directions within guidance angle range (use 5-degree steps for LaserScan)
  float step_size = 5.0f;
  float best_angle = 0.0f;
  float best_clearance = 0.0f;
  
  // First pass: find the direction with maximum clearance
  for (float offset = 0.0f; offset <= guidance_angle; offset += step_size) {
    for (float sign = -1.0f; sign <= 1.0f; sign += 2.0f) {
      if (offset == 0.0f && sign > 0.0f) continue; // Skip duplicate center check
      
      float test_angle_deg = desired_angle_deg + sign * offset;
      float obstacle_distance = getObstacleDistance(test_angle_deg);
      
      // For velocity mode, use a dynamic threshold
      float threshold = kCollisionThreshold;
      if (_state == State::VelocityControl) {
        threshold *= (1.0f + 0.5f * std::min(speed / kMaxSpeed, 1.0f));
      }
      
      if (obstacle_distance > threshold && obstacle_distance > best_clearance) {
        best_clearance = obstacle_distance;
        best_angle = test_angle_deg;
      }
    }
  }
  
  if (best_clearance > 0.0f) {
    // Safe direction found
    float test_angle_rad = best_angle * M_PI / 180.0f;
    float move_distance;
    
    if (_state == State::VelocityControl) {
      // For velocity mode, maintain the original speed
      move_distance = speed;
    } else {
      // For position mode, use the standard distance calculation
      move_distance = std::min(best_clearance * 0.5f, 2.0f);
    }
    
    return Eigen::Vector3f(
      move_distance * std::cos(test_angle_rad),
      move_distance * std::sin(test_angle_rad),
      desired_direction.z() // Maintain desired vertical movement
    );
  }
  
  return Eigen::Vector3f(0, 0, desired_direction.z()); // No safe direction found, but maintain altitude
}

float CollisionAutonomousMode::getObstacleDistance(float angle_deg) const
{
  if (!_laser_data_available) {
    return 1000.0f; // Assume clear if no data
  }
  
  // Convert LaserScan angles to degrees
  double angle_min_deg = _latest_laser_data.angle_min * 180.0 / M_PI;
  double angle_max_deg = _latest_laser_data.angle_max * 180.0 / M_PI;
  double angle_step_deg = _latest_laser_data.angle_increment * 180.0 / M_PI;
  
  // Normalize requested angle to [angle_min, angle_max]
  while (angle_deg < angle_min_deg) angle_deg += 360.0f;
  while (angle_deg >= angle_max_deg + 360.0f) angle_deg -= 360.0f;
  
  // Check if angle is within LaserScan range
  if (angle_deg < angle_min_deg || angle_deg > angle_max_deg) {
    return 1000.0f; // Outside sensor FOV, assume clear
  }
  
  // Find corresponding LaserScan index
  int laser_index = static_cast<int>((angle_deg - angle_min_deg) / angle_step_deg);
  laser_index = std::max(0, std::min(laser_index, static_cast<int>(_latest_laser_data.ranges.size()) - 1));
  
  double distance = _latest_laser_data.ranges[laser_index];
  
  // Handle invalid readings
  if (std::isinf(distance) || std::isnan(distance)) {
    return _latest_laser_data.range_max; // No obstacle detected
  }
  
  if (distance < _latest_laser_data.range_min || distance > _latest_laser_data.range_max) {
    return _latest_laser_data.range_max; // Invalid reading, assume clear
  }
  
  return static_cast<float>(distance);
}

bool CollisionAutonomousMode::isPathBlockedVDB(const Eigen::Vector3f& start, const Eigen::Vector3f& end)
{
  if (!_vdb_raytrace_client || !_vdb_raytrace_client->service_is_ready()) {
    RCLCPP_DEBUG_THROTTLE(_ros_node.get_logger(), *_ros_node.get_clock(), 5000, 
      "VDB raytrace service not ready");
    return false; // VDB service not ready
  }
  
  // Check transform availability first
  try {
    auto transform = _tf_buffer->lookupTransform("local_origin", "px4_body", 
                                               tf2::TimePointZero, tf2::durationFromSec(0.1));
    RCLCPP_DEBUG(_ros_node.get_logger(), "Transform available for raytrace");
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(_ros_node.get_logger(), *_ros_node.get_clock(), 2000,
      "Transform not available for raytrace: %s", ex.what());
    return false; // Skip this raytrace call
  }
  
  // Create single ray from start to end
  auto request = std::make_shared<vdb_mapping_interfaces::srv::BatchRaytrace::Request>();
  request->header.frame_id = "px4_body";
  request->header.stamp = rclcpp::Time(0); // Use latest available transform
  
  vdb_mapping_interfaces::msg::Ray ray;
  ray.origin.x = start.x();
  ray.origin.y = start.y();
  ray.origin.z = start.z();
  
  Eigen::Vector3f direction = (end - start).normalized();
  ray.direction.x = direction.x();
  ray.direction.y = direction.y();
  ray.direction.z = direction.z();
  ray.max_ray_length = (end - start).norm();
  
  request->rays.push_back(ray);
  
  // Synchronous call for path blocking check with longer timeout
  auto future = _vdb_raytrace_client->async_send_request(request);
  if (future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
    auto response = future.get();
    if (!response->successes.empty()) {
      RCLCPP_DEBUG(_ros_node.get_logger(), "Path blocked: %s", 
                   response->successes[0] ? "YES" : "NO");
      return response->successes[0]; // Hit = path blocked
    }
  } else {
    RCLCPP_WARN_THROTTLE(_ros_node.get_logger(), *_ros_node.get_clock(), 1000,
      "VDB raytrace timeout - assuming path clear");
  }
  
  return false; // Timeout or no hit = path clear
}

Eigen::Vector3f CollisionAutonomousMode::findSafeDirectionVDB(const Eigen::Vector3f& desired_direction)
{
  if (!_vehicle_local_position->positionXYValid()) {
    return Eigen::Vector3f::Zero();
  }
  
  Eigen::Vector3f current_pos = _vehicle_local_position->positionNed();
  float speed = desired_direction.norm();
  
  // Use smaller threshold for velocity mode
  float min_speed = (_state == State::VelocityControl) ? 0.05f : 0.1f;
  if (speed < min_speed) {
    return Eigen::Vector3f::Zero();
  }
  
  // Try different directions in 3D space
  float desired_yaw = std::atan2(desired_direction.y(), desired_direction.x());
  float guidance_angle = kGuidanceAngleDeg * M_PI / 180.0f;
  
  // Adjust guidance angle based on speed in velocity mode
  if (_state == State::VelocityControl) {
    guidance_angle *= (1.0f + 0.5f * std::min(speed / kMaxSpeed, 1.0f));
  }
  
  float best_clearance = 0.0f;
  Eigen::Vector3f best_direction = Eigen::Vector3f::Zero();
  
  // Test horizontal directions
  for (float yaw_offset = -guidance_angle; yaw_offset <= guidance_angle; yaw_offset += 10.0f * M_PI / 180.0f) {
    float test_yaw = desired_yaw + yaw_offset;
    
    // Test different altitudes too
    for (float z_offset = -0.5f; z_offset <= 0.5f; z_offset += 0.25f) {
      Eigen::Vector3f test_direction(
        speed * std::cos(test_yaw),
        speed * std::sin(test_yaw),
        desired_direction.z() + z_offset
      );
      
      Eigen::Vector3f test_end = current_pos + test_direction;
      
      if (!isPathBlockedVDB(current_pos, test_end)) {
        float clearance = getObstacleDistanceVDB(test_direction);
        if (clearance > best_clearance) {
          best_clearance = clearance;
          best_direction = test_direction;
        }
      }
    }
  }
  
  return best_direction;
}

float CollisionAutonomousMode::getObstacleDistanceVDB(const Eigen::Vector3f& /* direction */)
{
  if (!_vdb_data_available || !_vehicle_local_position->positionXYValid()) {
    return 1000.0f; // Assume clear if no data
  }
  
  Eigen::Vector3f current_pos = _vehicle_local_position->positionNed();
  float max_distance = 3.0f; // Maximum check distance
  
  // Check VDB service availability
  if (!_vdb_raytrace_client) {
    RCLCPP_WARN(_ros_node.get_logger(), "VDB raytrace client is null");
    return max_distance;
  }
  
  if (!_vdb_raytrace_client->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(_ros_node.get_logger(), *_ros_node.get_clock(), 2000, 
      "VDB service not ready for spherical raytrace");
    return max_distance;
  }
  
  // Create spherical rays around px4_body origin (0,0,0)
  std::vector<vdb_mapping_interfaces::msg::Ray> rays = generateSphericalRays(Eigen::Vector3f::Zero(), max_distance);
  
  // Prepare batch raytrace request - send rays directly in px4_body frame
  auto request = std::make_shared<vdb_mapping_interfaces::srv::BatchRaytrace::Request>();
  request->header.frame_id = "px4_body";
  request->header.stamp = _ros_node.get_clock()->now();
  request->rays = rays;
  
  RCLCPP_INFO(_ros_node.get_logger(), "Sending %zu rays in px4_body frame", request->rays.size());
  RCLCPP_INFO(_ros_node.get_logger(), "First ray: origin=(%.2f,%.2f,%.2f) dir=(%.2f,%.2f,%.2f) len=%.2f", 
              request->rays[0].origin.x, request->rays[0].origin.y, request->rays[0].origin.z,
              request->rays[0].direction.x, request->rays[0].direction.y, request->rays[0].direction.z,
              request->rays[0].max_ray_length);
  
  // Send batch raytrace request
  std::vector<bool> hits;
  std::vector<geometry_msgs::msg::Point> intersections;
  hits.resize(request->rays.size(), false);
  intersections.resize(request->rays.size());
  
  auto future = _vdb_raytrace_client->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
    auto response = future.get();
    
    RCLCPP_INFO(_ros_node.get_logger(), "VDB response: frame_id=%s, %zu results", 
                response->header.frame_id.c_str(), response->successes.size());
    
    if (response->successes.size() != request->rays.size()) {
      RCLCPP_WARN(_ros_node.get_logger(), "VDB returned %zu results for %zu rays", 
                  response->successes.size(), request->rays.size());
      return max_distance;
    }
    
    // Process raytrace results - results are in local_origin frame
    size_t hit_count = 0;
    for (size_t i = 0; i < response->successes.size(); ++i) {
      if (response->successes[i]) {
        hits[i] = true;
        intersections[i] = response->end_points[i];  // These are in local_origin frame
        hit_count++;
        
        // Debug first few hits
        if (hit_count <= 5) {
          RCLCPP_INFO(_ros_node.get_logger(), "Hit %zu at local_origin (%.2f, %.2f, %.2f)", hit_count,
                      response->end_points[i].x, response->end_points[i].y, response->end_points[i].z);
        }
      } else {
        // No hit - calculate end point in local_origin frame
        geometry_msgs::msg::Point end_local_origin;
        end_local_origin.x = response->end_points[i].x;
        end_local_origin.y = response->end_points[i].y;
        end_local_origin.z = response->end_points[i].z;
        intersections[i] = end_local_origin;
      }
    }
    RCLCPP_INFO(_ros_node.get_logger(), "VDB returned %zu hits out of %zu rays", 
                hit_count, response->successes.size());
  } else {
    RCLCPP_WARN_THROTTLE(_ros_node.get_logger(), *_ros_node.get_clock(), 1000,
      "VDB raytrace timeout - assuming clear path");
  }
  
  // Publish visualization 
  publishCollisionSphere(current_pos, rays, intersections, hits);
  
  RCLCPP_DEBUG(_ros_node.get_logger(), "VDB raytrace: %zu rays, %zu hits", 
               rays.size(), std::count(hits.begin(), hits.end(), true));
  
  return max_distance;  // Return max distance if no hits found
}

std::vector<vdb_mapping_interfaces::msg::Ray> CollisionAutonomousMode::generateSphericalRays(const Eigen::Vector3f& origin, float max_range) const
{
  std::vector<vdb_mapping_interfaces::msg::Ray> rays;
  
  // Generate dense spherical points using spherical coordinates
  std::vector<Eigen::Vector3f> directions;
  
  // Generate uniform sphere using Fibonacci spiral with deduplication
  const int num_points = 100;  // Reduced for testing
  const float golden_angle = M_PI * (3.0f - std::sqrt(5.0f));  // Golden angle in radians

  // Use set to detect duplicates (with some tolerance)
  std::set<std::tuple<int, int, int>> direction_set;
  const float scale = 100.0f;  // Scale for integer comparison

  for (int i = 0; i < num_points; i++) {
    float y = 1.0f - (2.0f * i) / (num_points - 1.0f);  // y goes from 1 to -1
    float radius = std::sqrt(1.0f - y * y);  // radius at y
    
    float theta = golden_angle * i;  // Golden angle increment
    
    // Convert to Cartesian coordinates
    Eigen::Vector3f dir(
      radius * std::cos(theta),  // x
      radius * std::sin(theta),  // y
      y                         // z
    );
    
    // Check for duplicates with tolerance
    auto scaled_dir = std::make_tuple(
      static_cast<int>(dir.x() * scale),
      static_cast<int>(dir.y() * scale),
      static_cast<int>(dir.z() * scale)
    );
    
    if (direction_set.insert(scaled_dir).second) {
      directions.push_back(dir);
    }
  }
  
  RCLCPP_INFO(_ros_node.get_logger(), "Generated %zu unique directions from %d points", 
              directions.size(), num_points);
  
  
  // Convert to rays
  for (const auto& dir : directions) {
    vdb_mapping_interfaces::msg::Ray ray;
    ray.origin.x = origin.x();
    ray.origin.y = origin.y();
    ray.origin.z = origin.z();
    
    ray.direction.x = dir.x();
    ray.direction.y = dir.y();
    ray.direction.z = dir.z();
    ray.max_ray_length = max_range;
    
    rays.push_back(ray);
  }
  
  return rays;
}

void CollisionAutonomousMode::publishCollisionSphere(const Eigen::Vector3f& origin, 
                                                     const std::vector<vdb_mapping_interfaces::msg::Ray>& rays,
                                                     const std::vector<geometry_msgs::msg::Point>& intersections,
                                                     const std::vector<bool>& hits)
{
  auto now = _ros_node.get_clock()->now();

  // Get transform from local_origin to px4_body frame for visualization
  geometry_msgs::msg::TransformStamped local_origin_to_body_transform;
  try {
    local_origin_to_body_transform = _tf_buffer->lookupTransform("px4_body", "local_origin",
                                          tf2::TimePointZero, tf2::durationFromSec(0.1));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(_ros_node.get_logger(), *_ros_node.get_clock(), 2000,
      "Cannot transform collision sphere: %s", ex.what());
    return;
  }
  
  // 1. Publish point cloud for hits/misses - transform to px4_body frame
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.header.frame_id = "px4_body";
  cloud.header.stamp = now.nanoseconds() / 1000;  // PCL uses microseconds
  
  for (size_t i = 0; i < rays.size() && i < hits.size(); ++i) {
    pcl::PointXYZRGB point;
    
    // Transform from local_origin to px4_body for visualization around drone
    geometry_msgs::msg::Point point_local = intersections[i];
    geometry_msgs::msg::Point point_body;
    tf2::doTransform(point_local, point_body, local_origin_to_body_transform);
    
    point.x = point_body.x;
    point.y = point_body.y;
    point.z = point_body.z;
    
    if (hits[i]) {
      point.r = 255; point.g = 0; point.b = 0;  // Red for hits
    } else {
      point.r = 0; point.g = 255; point.b = 0;  // Green for misses
    }
    
    cloud.points.push_back(point);
  }
  
  // Convert and publish point cloud
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = now;
  cloud_msg.header.frame_id = "px4_body";
  _collision_sphere_pub->publish(cloud_msg);

  // 2. Publish ray lines in px4_body frame
  visualization_msgs::msg::MarkerArray ray_markers;
  for (size_t i = 0; i < rays.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "px4_body";
    marker.header.stamp = now;
    marker.ns = "rays";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;  // Line width
    
    // Start point - center of drone (0,0,0 in px4_body frame)
    geometry_msgs::msg::Point start;
    start.x = 0.0;
    start.y = 0.0;
    start.z = 0.0;
    marker.points.push_back(start);
    
    // End point - transform from local_origin to px4_body
    geometry_msgs::msg::Point point_local = intersections[i];
    geometry_msgs::msg::Point end;
    tf2::doTransform(point_local, end, local_origin_to_body_transform);
    marker.points.push_back(end);
    
    if (hits[i]) {
      marker.color.r = 1.0;
      marker.color.a = 0.3;
    } else {
      marker.color.g = 1.0;
      marker.color.a = 0.1;
    }
    
    ray_markers.markers.push_back(marker);
  }
  _ray_lines_pub->publish(ray_markers);

  // 3. Publish hit voxels in px4_body frame
  visualization_msgs::msg::MarkerArray voxel_markers;
  for (size_t i = 0; i < hits.size(); ++i) {
    if (!hits[i] || i >= intersections.size()) continue;
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "px4_body";
    marker.header.stamp = now;
    marker.ns = "voxels";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Transform position from local_origin to px4_body frame
    geometry_msgs::msg::Point point_local;
    point_local.x = intersections[i].x;
    point_local.y = intersections[i].y;
    point_local.z = intersections[i].z;
    
    geometry_msgs::msg::Point point_body;
    tf2::doTransform(point_local, point_body, local_origin_to_body_transform);
    
    marker.pose.position.x = point_body.x;
    marker.pose.position.y = point_body.y;
    marker.pose.position.z = point_body.z;
    marker.pose.orientation.w = 1.0;
    
    // Size of voxel
    marker.scale.x = 0.1;  // 10cm voxels
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    
    // Red semi-transparent
    marker.color.r = 1.0;
    marker.color.a = 0.5;
    
    voxel_markers.markers.push_back(marker);
  }
  _hit_voxels_pub->publish(voxel_markers);
  
  RCLCPP_DEBUG(_ros_node.get_logger(), "Published collision sphere with %zu rays, %zu hits", 
               rays.size(), std::count(hits.begin(), hits.end(), true));
}