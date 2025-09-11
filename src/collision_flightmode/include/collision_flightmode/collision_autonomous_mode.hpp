/****************************************************************************
 * Copyright (c) 2024 Collision Autonomous Flight Mode.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/geometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/client.hpp>
#include <vdb_mapping_interfaces/srv/batch_raytrace.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <vector>
#include <memory>
#include <optional>

static const std::string kCollisionAwareName = "Collision Aware Auto";

using namespace px4_ros2::literals;

class CollisionAutonomousMode : public px4_ros2::ModeBase
{
public:
  explicit CollisionAutonomousMode(rclcpp::Node & node);

  void onActivate() override;
  void onDeactivate() override;
  void updateSetpoint(float dt_s) override;

private:
  enum class State
  {
    Idle = 0,
    NavigatingToWaypoint,
    AvoidingObstacle,
    MissionComplete,
    VelocityControl
  };

  // Callback functions
  void waypointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  
  // Navigation functions
  bool positionReached(const Eigen::Vector3f& target, float tolerance = 1.0f) const;
  Eigen::Vector3f calculateNavigationSetpoint();
  Eigen::Vector3f applyCollisionAvoidance(const Eigen::Vector3f& desired_setpoint);
  
  // Collision avoidance functions
  bool isPathBlocked(const Eigen::Vector3f& direction) const;
  Eigen::Vector3f findSafeDirection(const Eigen::Vector3f& desired_direction);
  float getObstacleDistance(float angle_deg) const;
  
  // VDB enhanced collision avoidance using batch raytrace
  bool isPathBlockedVDB(const Eigen::Vector3f& start, const Eigen::Vector3f& end);
  Eigen::Vector3f findSafeDirectionVDB(const Eigen::Vector3f& desired_direction);
  float getObstacleDistanceVDB(const Eigen::Vector3f& direction);
  std::vector<vdb_mapping_interfaces::msg::Ray> generateSphericalRays(const Eigen::Vector3f& origin, float max_range) const;
  void publishCollisionSphere(const Eigen::Vector3f& origin, const std::vector<vdb_mapping_interfaces::msg::Ray>& rays, 
                             const std::vector<geometry_msgs::msg::Point>& intersections, 
                             const std::vector<bool>& hits);

  // PX4 ROS2 components
  std::shared_ptr<px4_ros2::GotoSetpointType> _goto_setpoint;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
  
  // ROS2 subscriptions
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _waypoint_sub;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_sub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _laser_scan_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _pointcloud_sub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _occupancy_grid_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _twist_sub;
  
  // Visualization publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _collision_sphere_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _ray_lines_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _hit_voxels_pub;
  
  // Transform handling
  std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
  
  // VDB raytrace service client
  rclcpp::Client<vdb_mapping_interfaces::srv::BatchRaytrace>::SharedPtr _vdb_raytrace_client;
  
  // Navigation state
  State _state;
  std::vector<Eigen::Vector3f> _waypoints;
  size_t _current_waypoint_index;
  
  // Collision avoidance state
  sensor_msgs::msg::LaserScan _latest_laser_data;
  bool _laser_data_available;
  rclcpp::Time _last_laser_update;
  
  // VDB mapping integration
  sensor_msgs::msg::PointCloud2 _latest_pointcloud;
  nav_msgs::msg::OccupancyGrid _latest_occupancy_grid;
  bool _vdb_data_available;
  rclcpp::Time _last_vdb_update;
  
  // Parameters
  static constexpr float kWaypointTolerance = 1.0f;
  static constexpr float kMaxSpeed = 3.0f;
  static constexpr float kCollisionThreshold = 2.0f;
  static constexpr float kGuidanceAngleDeg = 30.0f;
  static constexpr double kLaserTimeoutSec = 0.5;
  
  // Velocity control state
  Eigen::Vector3f _current_velocity;
  rclcpp::Time _last_velocity_command;
  static constexpr double kVelocityTimeoutSec = 0.5;

  // Store node reference for subscriptions
  rclcpp::Node & _ros_node;
};