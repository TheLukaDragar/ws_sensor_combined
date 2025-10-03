/****************************************************************************
 * Copyright (c) 2024 PX4 TF Publisher.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <px4_ros_com/frame_transforms.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_ros2/odometry/local_position.hpp>

using namespace std::chrono_literals; // NOLINT

class PX4TFPublisher : public rclcpp::Node
{
public:
  explicit PX4TFPublisher()
  : Node("px4_tf_publisher_node"), 
    transform_stamped_(std::make_shared<geometry_msgs::msg::TransformStamped>()),
    odom_msg_(std::make_shared<nav_msgs::msg::Odometry>())
  {
    // Create TF broadcasters
    _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Use TRANSIENT_LOCAL durability for static TF (correct ROS2 standard)
    rclcpp::QoS static_tf_qos = rclcpp::QoS(100)
      .reliability(rclcpp::ReliabilityPolicy::Reliable)
      .durability(rclcpp::DurabilityPolicy::TransientLocal)  // Correct ROS2 standard for static TF
      .history(rclcpp::HistoryPolicy::KeepLast);
    
    _static_tf_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this, static_tf_qos);
      //_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

    //..ODOMETY ISPUBLISED SO WE HAVE A VALDI TRANSFORM BEEWEN LOCAL ORIGIN AND PX4 BODY!!!

    
    // Subscribe to PX4 vehicle odometry with optimized QoS for spherical interpolation
    // Larger buffer helps TF interpolation during fast motion
    // auto px4_qos = rclcpp::QoS(15)  // Increased buffer for better TF interpolation
    //   .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    //   .durability(rclcpp::DurabilityPolicy::Volatile)
    //   .history(rclcpp::HistoryPolicy::KeepLast);

    // // Create odometry publisher with reliable QoS
    // auto odom_qos = rclcpp::QoS(10)
    //   .reliability(rclcpp::ReliabilityPolicy::Reliable)
    //   .durability(rclcpp::DurabilityPolicy::Volatile)
    //   .history(rclcpp::HistoryPolicy::KeepLast);
    
    // _odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("/odom", odom_qos);
    
    // _vehicle_odometry_subscriber = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    //   "/px4_0/fmu/out/vehicle_odometry", px4_qos,
    //   std::bind(&PX4TFPublisher::vehicleOdometryCallback, this, std::placeholders::_1));

    // Publish static transforms for coordinate system setup
    publishStaticTransforms();

    RCLCPP_INFO(this->get_logger(), "PX4 TF Publisher initialized!");
    RCLCPP_INFO(this->get_logger(), "Publishing: local_origin -> px4_body transform");
    RCLCPP_INFO(this->get_logger(), "Publishing static transforms: World -> local_origin, px4_body -> ld19_frame, px4_body -> camera_frame");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: /px4_0/fmu/out/vehicle_odometry");
  }

private:
  void publishStaticTransforms()
  {
    // Use default StaticTransformBroadcaster QoS (same as Isaac)
    // Don't override QoS - let it use the standard static TF settings
    
    std::vector<geometry_msgs::msg::TransformStamped> static_transforms;
    
    // Create World to local_origin transform (identity transform)
    geometry_msgs::msg::TransformStamped world_to_local;
    world_to_local.header.stamp = this->get_clock()->now();
    world_to_local.header.frame_id = "World";  // Parent frame
    world_to_local.child_frame_id = "local_origin"; // Child frame
    world_to_local.transform.translation.x = 0.0;
    world_to_local.transform.translation.y = 0.0;
    world_to_local.transform.translation.z = 0.0;
    world_to_local.transform.rotation.x = 0.0;
    world_to_local.transform.rotation.y = 0.0;
    world_to_local.transform.rotation.z = 0.0;
    world_to_local.transform.rotation.w = 1.0;
    static_transforms.push_back(world_to_local);
    
    // Create debug frame to show PX4 coordinate system with same rotation as body
    geometry_msgs::msg::TransformStamped debug_frame;
    debug_frame.header.stamp = this->get_clock()->now();
    debug_frame.header.frame_id = "px4_body";  // Parent frame
    debug_frame.child_frame_id = "px4_body_ld19_frame"; // Debug child frame
    debug_frame.transform.translation.x = 0.0;  // Lidar position relative to body
    debug_frame.transform.translation.y = 0.0;
    debug_frame.transform.translation.z = 0.05; // 5cm above body
    
    // Match local_origin orientation with 90° rotation to align X
    Eigen::AngleAxisd yaw(M_PI_2, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q_ned(yaw);
    Eigen::Quaterniond q_enu = px4_ros_com::frame_transforms::ned_to_enu_orientation(q_ned);
    debug_frame.transform.rotation.x = q_enu.x();
    debug_frame.transform.rotation.y = q_enu.y();
    debug_frame.transform.rotation.z = q_enu.z();
    debug_frame.transform.rotation.w = q_enu.w();
    static_transforms.push_back(debug_frame);
    
    // Create transform from debug frame to final ld_frame
    geometry_msgs::msg::TransformStamped debug_to_ld;
    debug_to_ld.header.stamp = this->get_clock()->now();
    debug_to_ld.header.frame_id = "px4_body_ld19_frame";  // Parent frame
    debug_to_ld.child_frame_id = "ld19_frame"; // Final child frame
    debug_to_ld.transform.translation.x = 0.0;
    debug_to_ld.transform.translation.y = 0.0;
    debug_to_ld.transform.translation.z = 0.0; // No additional translation
    debug_to_ld.transform.rotation.x = 0.0;
    debug_to_ld.transform.rotation.y = 0.0;
    debug_to_ld.transform.rotation.z = 0.0;
    debug_to_ld.transform.rotation.w = 1.0;
    static_transforms.push_back(debug_to_ld);
    
    // Create transform from px4_body to camera_frame
    geometry_msgs::msg::TransformStamped body_to_camera;
    body_to_camera.header.stamp = this->get_clock()->now();
    body_to_camera.header.frame_id = "px4_body";  // Parent frame
    body_to_camera.child_frame_id = "sim_camera"; // Camera child frame
    body_to_camera.transform.translation.x = 0.11;  // 11cm forward from body
    body_to_camera.transform.translation.y = 0.0;
    body_to_camera.transform.translation.z = 0.0;   // Same height as body
    
    // Camera orientation: +90° Yaw + 90° Pitch (removed the 180° twist that made image upside down)
    // Forward-facing camera without upside-down image
    Eigen::AngleAxisd camera_yaw(M_PI_2, Eigen::Vector3d::UnitZ());    // +90° around Z-axis 
    Eigen::AngleAxisd camera_pitch(M_PI_2, Eigen::Vector3d::UnitY());  // 90° around Y-axis to lift from ground
    Eigen::Quaterniond q_camera = camera_pitch * camera_yaw;  // Apply yaw, then pitch (no twist!)
    body_to_camera.transform.rotation.x = q_camera.x();
    body_to_camera.transform.rotation.y = q_camera.y();
    body_to_camera.transform.rotation.z = q_camera.z();
    body_to_camera.transform.rotation.w = q_camera.w();
    static_transforms.push_back(body_to_camera);
    
    // Publish all static transforms
    _static_tf_broadcaster->sendTransform(static_transforms);
    
    RCLCPP_INFO(this->get_logger(), "Published static transforms:");
    RCLCPP_INFO(this->get_logger(), "px4_body -> px4_body_ld19_frame with transform [0, 0, 0.05]");
    RCLCPP_INFO(this->get_logger(), "px4_body -> sim_camera with transform [0.11, 0, 0] and rotation Yaw(+90°) * Pitch(90°)");
  
  }

  void vehicleOdometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    // Use pre-allocated transform message for performance
    auto& transform_stamped = *transform_stamped_;
    
    // Header
    transform_stamped.header.stamp = this->get_clock()->now();
    transform_stamped.header.frame_id = "local_origin";  // Parent frame (world reference)
    transform_stamped.child_frame_id = "px4_body";       // Child frame (PX4 vehicle frame, renamed to avoid Isaac conflicts)
    
    // Convert PX4 NED position to ROS ENU using PX4's frame transform library
    Eigen::Vector3d pos_ned(msg->position[0], msg->position[1], msg->position[2]);
    Eigen::Vector3d pos_enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(pos_ned);
    
    transform_stamped.transform.translation.x = pos_enu.x();  // East
    transform_stamped.transform.translation.y = pos_enu.y();  // North  
    transform_stamped.transform.translation.z = pos_enu.z();  // Up
    
    // Convert PX4 NED quaternion to ROS ENU using PX4's frame transform library
    // PX4 quaternion order is [w, x, y, z], construct Eigen quaternion as (w, x, y, z)
    Eigen::Quaterniond q_ned(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    Eigen::Quaterniond q_enu = px4_ros_com::frame_transforms::ned_to_enu_orientation(q_ned);
    
    transform_stamped.transform.rotation.x = q_enu.x();
    transform_stamped.transform.rotation.y = q_enu.y(); 
    transform_stamped.transform.rotation.z = q_enu.z();
    transform_stamped.transform.rotation.w = q_enu.w();
    
    // Broadcast the transform
    _tf_broadcaster->sendTransform(transform_stamped);

    // Publish odometry message aligned with ld19_frame (without height offset)
    auto& odom = *odom_msg_;
    odom.header.stamp = transform_stamped.header.stamp;
    odom.header.frame_id = "local_origin";  // Same as TF
    odom.child_frame_id = "px4_body";       // Same as our TF frame

    // Use the same position but remove the z-offset of 0.05m
    odom.pose.pose.position.x = transform_stamped.transform.translation.x;
    odom.pose.pose.position.y = transform_stamped.transform.translation.y;
    odom.pose.pose.position.z = transform_stamped.transform.translation.z;

    // Use the same orientation as the transform - we're publishing in px4_body frame
    odom.pose.pose.orientation = transform_stamped.transform.rotation;

    // Set twist from PX4 velocity (convert NED to ENU)
    Eigen::Vector3d vel_ned(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
    Eigen::Vector3d vel_enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(vel_ned);
    odom.twist.twist.linear.x = vel_enu.x();
    odom.twist.twist.linear.y = vel_enu.y();
    odom.twist.twist.linear.z = vel_enu.z();

    // Angular velocity (convert NED to ENU)
    Eigen::Vector3d angular_ned(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]);
    Eigen::Vector3d angular_enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(angular_ned);
    odom.twist.twist.angular.x = angular_enu.x();
    odom.twist.twist.angular.y = angular_enu.y();
    odom.twist.twist.angular.z = angular_enu.z();

    // Publish the odometry message
    _odom_publisher->publish(odom);
    
    // Smart debug output - more frequent during fast motion for spherical interpolation debugging
    static int count = 0;
    static auto last_log_time = this->get_clock()->now();
    auto now = this->get_clock()->now();
    
    // Calculate motion magnitude for adaptive logging
    double motion_magnitude = std::sqrt(pos_enu.x()*pos_enu.x() + pos_enu.y()*pos_enu.y() + pos_enu.z()*pos_enu.z());
    
    // Adaptive logging: more frequent during fast motion (important for spherical interpolation)
    int log_interval = motion_magnitude > 1.0 ? 25 : 75;  // 25 msgs (~2Hz) when moving fast, 75 msgs (~0.7Hz) when slow
    
    if (++count % log_interval == 0) {
      double dt = (now - last_log_time).seconds();
      RCLCPP_INFO(this->get_logger(), 
        "[TF@%.1fHz] Motion:%.2fm ENU[%.2f,%.2f,%.2f] NED[%.2f,%.2f,%.2f] Q_ENU[%.2f,%.2f,%.2f,%.2f]",
        log_interval/dt, motion_magnitude,
        pos_enu.x(), pos_enu.y(), pos_enu.z(),
        msg->position[0], msg->position[1], msg->position[2],
        q_enu.w(), q_enu.x(), q_enu.y(), q_enu.z());
      last_log_time = now;
    }
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _vehicle_odometry_subscriber;
  std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> _static_tf_broadcaster;
  
  // Pre-allocated transform message for performance (avoid repeated allocations)
  std::shared_ptr<geometry_msgs::msg::TransformStamped> transform_stamped_;
  std::shared_ptr<nav_msgs::msg::Odometry> odom_msg_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_publisher;
};