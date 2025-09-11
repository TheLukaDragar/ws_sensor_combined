#pragma once

/**\file laser_scan_to_pointcloud_assembler.h
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <macros>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </macros>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <map>
#include <cmath>

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/timesync_status.hpp>

// external libs includes

// project includes
#include <laserscan_to_pointcloud/laserscan_to_ros_pointcloud.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


namespace laserscan_to_pointcloud {
// ###################################################################   laser_scan_to_pointcloud_assembler   ##################################################################
/**
 * \brief Description...
 */
class LaserScanToPointcloudAssembler {
	// ========================================================================   <public-section>   ===========================================================================
	public:
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <typedefs>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </typedefs>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <enums>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </enums>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constants>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constants>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		LaserScanToPointcloudAssembler(std::shared_ptr<rclcpp::Node> node);
		virtual ~LaserScanToPointcloudAssembler();

		void setupLaserScansSubscribers(std::string laser_scan_topics);
		void setupRecoveryInitialPose();
		void startAssemblingLaserScans();
		void stopAssemblingLaserScans();
		void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan);
		void adjustAssemblyConfiguration(const geometry_msgs::msg::Vector3& linear_velocity, const geometry_msgs::msg::Vector3& angular_velocity);
		void adjustAssemblyConfigurationFromTwist(const geometry_msgs::msg::Twist::SharedPtr twist);
		void adjustAssemblyConfigurationFromOdometry(const nav_msgs::msg::Odometry::SharedPtr odometry);
		void adjustAssemblyConfigurationFromIMU(const sensor_msgs::msg::Imu::SharedPtr imu);
		void adjustAssemblyConfigurationFromPX4Odometry(const px4_msgs::msg::VehicleOdometry::SharedPtr px4_odometry);
		void adjustAssemblyConfigurationFromPX4Attitude(const px4_msgs::msg::VehicleAttitude::SharedPtr px4_attitude);
		
		// Dynamic parameter callback
		rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <LaserScanToPointcloudAssembler-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </LaserScanToPointcloudAssembler-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <gets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </gets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <sets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </sets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// ========================================================================   </public-section>   ==========================================================================

	// ========================================================================   <protected-section>   ========================================================================
	protected:
	// ========================================================================   </protected-section>  ========================================================================

	// ========================================================================   <private-section>   ==========================================================================
	private:
		// communication fields (must be first since other members depend on it)
		std::shared_ptr<rclcpp::Node> node_;

		// assembler config fields
		std::string laser_scan_topics_;
		std::string pointcloud_publish_topic_;
		int number_of_scans_to_assemble_per_cloud_;
		rclcpp::Duration timeout_for_cloud_assembly_{rclcpp::Duration::from_seconds(0.0)};

		// dynamic reconfiguration based on twist messages
		int min_number_of_scans_to_assemble_per_cloud_;
		int max_number_of_scans_to_assemble_per_cloud_;
		double min_timeout_seconds_for_cloud_assembly_;
		double max_timeout_seconds_for_cloud_assembly_;

		double max_linear_velocity_;
		double max_angular_velocity_;

		// state fields (must be before laserscan_to_pointcloud_ due to initialization order)
		rclcpp::Time imu_last_message_stamp_;
		size_t number_droped_laserscans_;
		bool timeout_for_cloud_assembly_reached_;
		geometry_msgs::msg::Vector3 imu_linear_velocity;

		// laserscan_to_pointcloud_ config fields (must be after node_ and state fields)
		LaserScanToROSPointcloud laserscan_to_pointcloud_;
		bool include_laser_intensity_;
		bool enforce_reception_of_laser_scans_in_all_topics_;
		std::map<std::string, sensor_msgs::msg::LaserScan::SharedPtr> laser_scans_for_each_topic_frame_id_;
		std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> laserscan_subscribers_;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
		rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
		rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odometry_subscriber_;
		rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr px4_attitude_subscriber_;
		
		// Parameter callback handle for dynamic reconfiguration
		rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
	// ========================================================================   </private-section>  ==========================================================================
};
} /* namespace laserscan_to_pointcloud */
