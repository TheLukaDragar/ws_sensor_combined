/**\file laser_scan_to_pointcloud_assembler.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <laserscan_to_pointcloud/laserscan_to_pointcloud_assembler.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace laserscan_to_pointcloud {
// =============================================================================  <public-section>   ===========================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
LaserScanToPointcloudAssembler::LaserScanToPointcloudAssembler(std::shared_ptr<rclcpp::Node> node) :
		node_(node),
		imu_last_message_stamp_(rclcpp::Time(0)),
		number_droped_laserscans_(0),
		timeout_for_cloud_assembly_reached_(false),
		laserscan_to_pointcloud_(node) {

	// Declare parameters with default values (keeping original defaults)
	node_->declare_parameter("laser_scan_topics", std::string("/ld19_sim/scan"));
	node_->declare_parameter("pointcloud_publish_topic", std::string("ambient_pointcloud"));
	node_->declare_parameter("enforce_reception_of_laser_scans_in_all_topics", true);
	node_->declare_parameter("include_laser_intensity", true);
	node_->declare_parameter("number_of_scans_to_assemble_per_cloud", 20);  // Increased for 3D effect
	node_->declare_parameter("timeout_for_cloud_assembly", 1.0);
	node_->declare_parameter("min_number_of_scans_to_assemble_per_cloud", 5);
	node_->declare_parameter("max_number_of_scans_to_assemble_per_cloud", 50);
	node_->declare_parameter("min_timeout_seconds_for_cloud_assembly", 0.3);
	node_->declare_parameter("max_timeout_seconds_for_cloud_assembly", 2.0);
	node_->declare_parameter("max_linear_velocity", 2.0);  // Higher for drones
	node_->declare_parameter("max_angular_velocity", 1.0); // Higher for drones
	
	// Core algorithm parameters - KEEP THESE THE SAME to preserve algorithm
	node_->declare_parameter("target_frame", std::string("map"));
	node_->declare_parameter("laser_frame", std::string(""));
	node_->declare_parameter("motion_estimation_source_frame_id", std::string(""));
	node_->declare_parameter("motion_estimation_target_frame_id", std::string(""));
	node_->declare_parameter("number_of_tf_queries_for_spherical_interpolation", 4);
	node_->declare_parameter("tf_lookup_timeout", 0.15);
	node_->declare_parameter("min_range_cutoff_percentage_offset", 1.05);
	node_->declare_parameter("max_range_cutoff_percentage_offset", 0.95);
	node_->declare_parameter("remove_invalid_measurements", true);
	
	// PX4 specific parameters
	node_->declare_parameter("px4_odometry_topic", std::string("/px4_0/fmu/out/vehicle_odometry"));
	node_->declare_parameter("px4_attitude_topic", std::string("/px4_0/fmu/out/vehicle_attitude"));
	node_->declare_parameter("use_px4_motion_compensation", true);

	// Get parameters
	laser_scan_topics_ = node_->get_parameter("laser_scan_topics").as_string();
	pointcloud_publish_topic_ = node_->get_parameter("pointcloud_publish_topic").as_string();
	enforce_reception_of_laser_scans_in_all_topics_ = node_->get_parameter("enforce_reception_of_laser_scans_in_all_topics").as_bool();
	include_laser_intensity_ = node_->get_parameter("include_laser_intensity").as_bool();
	number_of_scans_to_assemble_per_cloud_ = node_->get_parameter("number_of_scans_to_assemble_per_cloud").as_int();
	double timeout_for_cloud_assembly = node_->get_parameter("timeout_for_cloud_assembly").as_double();
	timeout_for_cloud_assembly_ = rclcpp::Duration::from_nanoseconds(timeout_for_cloud_assembly * 1e9);
	
	min_number_of_scans_to_assemble_per_cloud_ = node_->get_parameter("min_number_of_scans_to_assemble_per_cloud").as_int();
	max_number_of_scans_to_assemble_per_cloud_ = node_->get_parameter("max_number_of_scans_to_assemble_per_cloud").as_int();
	min_timeout_seconds_for_cloud_assembly_ = node_->get_parameter("min_timeout_seconds_for_cloud_assembly").as_double();
	max_timeout_seconds_for_cloud_assembly_ = node_->get_parameter("max_timeout_seconds_for_cloud_assembly").as_double();
	max_linear_velocity_ = node_->get_parameter("max_linear_velocity").as_double();
	max_angular_velocity_ = node_->get_parameter("max_angular_velocity").as_double();

	// Configure core algorithm - PRESERVE THESE SETTINGS
	laserscan_to_pointcloud_.setIncludeLaserIntensity(include_laser_intensity_);
	
	std::string target_frame_id = node_->get_parameter("target_frame").as_string();
	laserscan_to_pointcloud_.setTargetFrame(target_frame_id);
	
	std::string motion_estimation_source_frame_id = node_->get_parameter("motion_estimation_source_frame_id").as_string();
	std::string motion_estimation_target_frame_id = node_->get_parameter("motion_estimation_target_frame_id").as_string();
	laserscan_to_pointcloud_.setMotionEstimationSourceFrame(motion_estimation_source_frame_id);
	laserscan_to_pointcloud_.setMotionEstimationTargetFrame(motion_estimation_target_frame_id);
	
	std::string laser_frame_id = node_->get_parameter("laser_frame").as_string();
	laserscan_to_pointcloud_.setLaserFrame(laser_frame_id);
	
	int number_of_tf_queries = node_->get_parameter("number_of_tf_queries_for_spherical_interpolation").as_int();
	if (number_of_tf_queries > 1) {
		RCLCPP_INFO(node_->get_logger(), "Laser assembler is using %d TFs inside laser scan time to perform spherical interpolation", number_of_tf_queries);
	}
	laserscan_to_pointcloud_.setNumberOfTfQueriesForSphericalInterpolation(number_of_tf_queries);
	
	double tf_lookup_timeout = node_->get_parameter("tf_lookup_timeout").as_double();
	laserscan_to_pointcloud_.setTFLookupTimeout(tf_lookup_timeout);
	
	double min_range_cutoff = node_->get_parameter("min_range_cutoff_percentage_offset").as_double();
	double max_range_cutoff = node_->get_parameter("max_range_cutoff_percentage_offset").as_double();
	laserscan_to_pointcloud_.setMinRangeCutoffPercentageOffset(min_range_cutoff);
	laserscan_to_pointcloud_.setMaxRangeCutoffPercentageOffset(max_range_cutoff);
	
	bool remove_invalid = node_->get_parameter("remove_invalid_measurements").as_bool();
	laserscan_to_pointcloud_.setRemoveInvalidMeasurements(remove_invalid);

	// Setup PX4 subscribers for motion compensation
	std::string px4_odometry_topic = node_->get_parameter("px4_odometry_topic").as_string();
	std::string px4_attitude_topic = node_->get_parameter("px4_attitude_topic").as_string();
	bool use_px4_motion_compensation = node_->get_parameter("use_px4_motion_compensation").as_bool();
	
	if (use_px4_motion_compensation) {
		// Use best effort QoS for PX4 topics
		auto px4_qos = rclcpp::QoS(5).reliability(rclcpp::ReliabilityPolicy::BestEffort);
		
		px4_odometry_subscriber_ = node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
			px4_odometry_topic, px4_qos,
			std::bind(&LaserScanToPointcloudAssembler::adjustAssemblyConfigurationFromPX4Odometry, this, std::placeholders::_1));
		
		px4_attitude_subscriber_ = node_->create_subscription<px4_msgs::msg::VehicleAttitude>(
			px4_attitude_topic, px4_qos,
			std::bind(&LaserScanToPointcloudAssembler::adjustAssemblyConfigurationFromPX4Attitude, this, std::placeholders::_1));
		
		// Note: Removed manual PX4 timesync handling as XRCE-DDS bridge handles time sync automatically
		
		RCLCPP_INFO(node_->get_logger(), "PX4 motion compensation enabled:");
		RCLCPP_INFO(node_->get_logger(), "  Odometry topic: %s", px4_odometry_topic.c_str());
		RCLCPP_INFO(node_->get_logger(), "  Attitude topic: %s", px4_attitude_topic.c_str());
	}

	// Set up dynamic parameter callback for real-time parameter updates
	param_callback_handle_ = node_->add_on_set_parameters_callback(
		std::bind(&LaserScanToPointcloudAssembler::parametersCallback, this, std::placeholders::_1));

	RCLCPP_INFO(node_->get_logger(), "LaserScan to PointCloud3D assembler initialized");
	RCLCPP_INFO(node_->get_logger(), "  Target frame: %s", target_frame_id.c_str());
	RCLCPP_INFO(node_->get_logger(), "  Spherical interpolation points: %d", number_of_tf_queries);
	RCLCPP_INFO(node_->get_logger(), "  Scans per cloud: %d", number_of_scans_to_assemble_per_cloud_);
	RCLCPP_INFO(node_->get_logger(), "  Dynamic parameter updates: ENABLED");
}

LaserScanToPointcloudAssembler::~LaserScanToPointcloudAssembler() {	}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <LaserScanToPointcloudAssembler-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void LaserScanToPointcloudAssembler::setupLaserScansSubscribers(std::string laser_scan_topics) {
	std::replace(laser_scan_topics.begin(), laser_scan_topics.end(), '+', ' ');

	std::stringstream ss(laser_scan_topics);
	std::string topic_name;

	while (ss >> topic_name && !topic_name.empty()) {
		auto laserscan_subscriber = node_->create_subscription<sensor_msgs::msg::LaserScan>(
			topic_name, 5,
			std::bind(&LaserScanToPointcloudAssembler::processLaserScan, this, std::placeholders::_1));
		laserscan_subscribers_.push_back(laserscan_subscriber);
		RCLCPP_INFO(node_->get_logger(), "Adding %s to the list of LaserScan topics to assemble", topic_name.c_str());
	}

	if (number_of_scans_to_assemble_per_cloud_ < (int)laserscan_subscribers_.size()) {
		number_of_scans_to_assemble_per_cloud_ = (int)laserscan_subscribers_.size();
	}

	if (min_number_of_scans_to_assemble_per_cloud_ < (int)laserscan_subscribers_.size()) {
		min_number_of_scans_to_assemble_per_cloud_ = (int)laserscan_subscribers_.size();
	}

	if (max_number_of_scans_to_assemble_per_cloud_ <= (int)min_number_of_scans_to_assemble_per_cloud_) {
		max_number_of_scans_to_assemble_per_cloud_ = (int)(min_number_of_scans_to_assemble_per_cloud_ * 2);
	}
}


void LaserScanToPointcloudAssembler::setupRecoveryInitialPose() {
	// Declare recovery parameters
	node_->declare_parameter("recovery_frame", std::string("odom"));
	node_->declare_parameter("initial_recovery_transform_in_base_link_to_target", true);
	node_->declare_parameter("base_link_frame_id", std::string("base_footprint"));
	node_->declare_parameter("recovery_to_target_frame_transform_initial_x", 0.0);
	node_->declare_parameter("recovery_to_target_frame_transform_initial_y", 0.0);
	node_->declare_parameter("recovery_to_target_frame_transform_initial_z", 0.0);
	node_->declare_parameter("recovery_to_target_frame_transform_initial_roll", 0.0);
	node_->declare_parameter("recovery_to_target_frame_transform_initial_pitch", 0.0);
	node_->declare_parameter("recovery_to_target_frame_transform_initial_yaw", 0.0);

	// Get parameters
	std::string recovery_frame_id = node_->get_parameter("recovery_frame").as_string();
	bool initial_recovery_transform_in_base_link_to_target = node_->get_parameter("initial_recovery_transform_in_base_link_to_target").as_bool();
	std::string base_link_frame_id = node_->get_parameter("base_link_frame_id").as_string();
	double x = node_->get_parameter("recovery_to_target_frame_transform_initial_x").as_double();
	double y = node_->get_parameter("recovery_to_target_frame_transform_initial_y").as_double();
	double z = node_->get_parameter("recovery_to_target_frame_transform_initial_z").as_double();
	double roll = node_->get_parameter("recovery_to_target_frame_transform_initial_roll").as_double();
	double pitch = node_->get_parameter("recovery_to_target_frame_transform_initial_pitch").as_double();
	double yaw = node_->get_parameter("recovery_to_target_frame_transform_initial_yaw").as_double();

	tf2::Quaternion orientation;
	orientation.setRPY(roll, pitch, yaw);
	orientation.normalize();
	tf2::Transform recovery_to_target_frame_transform(orientation, tf2::Vector3(x, y, z));

	if (initial_recovery_transform_in_base_link_to_target && !recovery_frame_id.empty() && !base_link_frame_id.empty()) {
		rclcpp::Time start_time = node_->get_clock()->now();
		rclcpp::Time end_time = start_time + rclcpp::Duration::from_nanoseconds(10 * 1e9);
		rclcpp::Duration wait_duration = rclcpp::Duration::from_nanoseconds(5 * 1e6);

		bool success = false;
		while (node_->get_clock()->now() < end_time) {
			tf2::Transform transform_recovery_to_base_link;
			if (laserscan_to_pointcloud_.getTfCollector().lookForTransform(transform_recovery_to_base_link, base_link_frame_id, recovery_frame_id, node_->get_clock()->now(), laserscan_to_pointcloud_.getTfLookupTimeout())) {
				recovery_to_target_frame_transform = recovery_to_target_frame_transform * transform_recovery_to_base_link;
				success = true;
				break;
			}
			rclcpp::sleep_for(std::chrono::nanoseconds(wait_duration.nanoseconds()));
		}
		if (!success) {
			RCLCPP_WARN(node_->get_logger(), "Failed to correct assembler recovery initial pose");
		}
	}

	if (!recovery_frame_id.empty()) {
		tf2::Quaternion recovery_to_target_frame_transform_q = recovery_to_target_frame_transform.getRotation().normalize();
		RCLCPP_INFO(node_->get_logger(), "Setting assembler recovery initial pose [ x: %.3f y: %.3f z: %.3f | qx: %.3f qy: %.3f qz: %.3f qw: %.3f ]",
			recovery_to_target_frame_transform.getOrigin().getX(),
			recovery_to_target_frame_transform.getOrigin().getY(),
			recovery_to_target_frame_transform.getOrigin().getZ(),
			recovery_to_target_frame_transform_q.getX(),
			recovery_to_target_frame_transform_q.getY(),
			recovery_to_target_frame_transform_q.getZ(),
			recovery_to_target_frame_transform_q.getW());
		laserscan_to_pointcloud_.setRecoveryFrame(recovery_frame_id, recovery_to_target_frame_transform);
	}
}


void LaserScanToPointcloudAssembler::startAssemblingLaserScans() {
	setupRecoveryInitialPose();
	pointcloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_publish_topic_, 10);
	setupLaserScansSubscribers(laser_scan_topics_);
	
	RCLCPP_INFO(node_->get_logger(), "Started assembling laser scans to point clouds");
	RCLCPP_INFO(node_->get_logger(), "  Publishing to: %s", pointcloud_publish_topic_.c_str());
}


void LaserScanToPointcloudAssembler::stopAssemblingLaserScans() {
	// In ROS2, subscription destruction happens automatically
	laserscan_subscribers_.clear();
	pointcloud_publisher_.reset();
	
	RCLCPP_INFO(node_->get_logger(), "Stopped assembling laser scans");
}


void LaserScanToPointcloudAssembler::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan) {
	// CORE ALGORITHM PRESERVED - Only changing ROS1->ROS2 API calls
	int number_of_scans_in_current_pointcloud = (int)laserscan_to_pointcloud_.getNumberOfScansAssembledInCurrentPointcloud();
	if ((number_of_scans_in_current_pointcloud == 0 && laserscan_to_pointcloud_.getNumberOfPointcloudsCreated() == 0)
			|| number_of_scans_in_current_pointcloud >= number_of_scans_to_assemble_per_cloud_
			|| timeout_for_cloud_assembly_reached_) {
		laserscan_to_pointcloud_.setIncludeLaserIntensity(include_laser_intensity_);
		laserscan_to_pointcloud_.initNewPointCloud(laser_scan->ranges.size() * number_of_scans_to_assemble_per_cloud_);
		laser_scans_for_each_topic_frame_id_.clear();
		timeout_for_cloud_assembly_reached_ = false;

		RCLCPP_INFO(node_->get_logger(), "Initializing new point cloud");
	}

	number_of_scans_in_current_pointcloud = (int)laserscan_to_pointcloud_.getNumberOfScansAssembledInCurrentPointcloud();

	std::string laser_frame = laserscan_to_pointcloud_.getLaserFrame().empty() ? laser_scan->header.frame_id : laserscan_to_pointcloud_.getLaserFrame();

	RCLCPP_INFO(node_->get_logger(), "%s laser scan %d in frame %s with %zu points to a point cloud with %zu points in frame %s",
		(enforce_reception_of_laser_scans_in_all_topics_ ? "Caching" : "Adding"),
		number_of_scans_in_current_pointcloud + static_cast<int>(laser_scans_for_each_topic_frame_id_.size()),
		laser_frame.c_str(), laser_scan->ranges.size(), 
		laserscan_to_pointcloud_.getNumberOfPointsInCloud(),
		laserscan_to_pointcloud_.getTargetFrame().c_str());

	if (enforce_reception_of_laser_scans_in_all_topics_) {
		if (laser_scans_for_each_topic_frame_id_.find(laser_scan->header.frame_id) != laser_scans_for_each_topic_frame_id_.end())
			RCLCPP_WARN(node_->get_logger(), "Discarding previously cached laser scan for frame %s", laser_scan->header.frame_id.c_str());

		laser_scans_for_each_topic_frame_id_[laser_scan->header.frame_id] = laser_scan;

		if (laser_scans_for_each_topic_frame_id_.size() >= laserscan_subscribers_.size()) {
			for (auto it = laser_scans_for_each_topic_frame_id_.begin(); it != laser_scans_for_each_topic_frame_id_.end(); ++it) {
				// CORE ALGORITHM CALL - spherical linear interpolation preserved exactly
				if (!laserscan_to_pointcloud_.integrateLaserScanWithShpericalLinearInterpolation(it->second)) {
					RCLCPP_WARN(node_->get_logger(), "Dropped LaserScan with %zu points because of missing TFs between [%s] and [%s] (dropped %zu LaserScans so far)",
						it->second->ranges.size(), laser_frame.c_str(), laserscan_to_pointcloud_.getTargetFrame().c_str(), ++number_droped_laserscans_);
				}
			}

			laser_scans_for_each_topic_frame_id_.clear();
		}
	} else if (!laserscan_to_pointcloud_.integrateLaserScanWithShpericalLinearInterpolation(laser_scan)) {
		// CORE ALGORITHM CALL - spherical linear interpolation preserved exactly  
		RCLCPP_WARN(node_->get_logger(), "Dropped LaserScan with %zu points because of missing TFs between [%s] and [%s] (dropped %zu LaserScans so far)",
			laser_scan->ranges.size(), laser_frame.c_str(), laserscan_to_pointcloud_.getTargetFrame().c_str(), ++number_droped_laserscans_);
	}

	// ROS2 fix: Compare message timestamps (same time source) instead of mixing system clock with message time
	// In ROS1 this was: (ros::Time::now() - pointcloud_stamp) > timeout (worked because ros::Time::now() respected /use_sim_time)
	// In ROS2: node_->get_clock()->now() might use different time source than message timestamps
	rclcpp::Time pointcloud_stamp(laserscan_to_pointcloud_.getPointcloud()->header.stamp.sec, 
								  laserscan_to_pointcloud_.getPointcloud()->header.stamp.nanosec);
	rclcpp::Time current_laser_time(laser_scan->header.stamp.sec, laser_scan->header.stamp.nanosec);
	
	// Compare message timestamps directly (both from same time source)
	timeout_for_cloud_assembly_reached_ = (current_laser_time - pointcloud_stamp) > timeout_for_cloud_assembly_;
	
	number_of_scans_in_current_pointcloud = (int)laserscan_to_pointcloud_.getNumberOfScansAssembledInCurrentPointcloud();
	if ((number_of_scans_in_current_pointcloud >= number_of_scans_to_assemble_per_cloud_ || timeout_for_cloud_assembly_reached_) && laserscan_to_pointcloud_.getNumberOfPointsInCloud() > 0) {
		// Calculate scan duration - ALGORITHM PRESERVED
		rclcpp::Duration scan_duration = rclcpp::Duration::from_nanoseconds((laser_scan->ranges.size() - 1) * laser_scan->time_increment * 1e9);
		rclcpp::Time laser_stamp(laser_scan->header.stamp.sec, laser_scan->header.stamp.nanosec);
		rclcpp::Time final_stamp = laser_stamp + scan_duration;
		
		// Set final timestamp
		laserscan_to_pointcloud_.getPointcloud()->header.stamp.sec = final_stamp.seconds();
		laserscan_to_pointcloud_.getPointcloud()->header.stamp.nanosec = final_stamp.nanoseconds() % 1000000000UL;
		
		// Publish the 3D point cloud
		pointcloud_publisher_->publish(*laserscan_to_pointcloud_.getPointcloud());

		RCLCPP_INFO(node_->get_logger(), "Publishing cloud with %u points assembled from %d LaserScans%s",
			(laserscan_to_pointcloud_.getPointcloud()->width * laserscan_to_pointcloud_.getPointcloud()->height),
			number_of_scans_in_current_pointcloud,
			(timeout_for_cloud_assembly_reached_ ? " (timeout reached)" : ""));
	}
}


void LaserScanToPointcloudAssembler::adjustAssemblyConfiguration(const geometry_msgs::msg::Vector3& linear_velocity, const geometry_msgs::msg::Vector3& angular_velocity) {
	// CORE VELOCITY-BASED ASSEMBLY ADJUSTMENT ALGORITHM PRESERVED
	double inverse_linear_velocity = max_linear_velocity_ - std::min(std::sqrt(linear_velocity.x * linear_velocity.x + linear_velocity.y * linear_velocity.y + linear_velocity.z * linear_velocity.z), max_linear_velocity_);
	double inverse_angular_velocity = max_angular_velocity_ - std::min(std::sqrt(angular_velocity.x * angular_velocity.x + angular_velocity.y * angular_velocity.y + angular_velocity.z * angular_velocity.z), max_angular_velocity_);

	double linear_velocity_to_number_scans_ratio = (max_number_of_scans_to_assemble_per_cloud_ - min_number_of_scans_to_assemble_per_cloud_) / max_linear_velocity_;
	double angular_velocity_to_number_scans_ratio = (max_number_of_scans_to_assemble_per_cloud_ - min_number_of_scans_to_assemble_per_cloud_) / max_angular_velocity_;
	double number_scans_linear_velocity = min_number_of_scans_to_assemble_per_cloud_ + linear_velocity_to_number_scans_ratio * inverse_linear_velocity;
	double number_scans_angular_velocity = min_number_of_scans_to_assemble_per_cloud_ + angular_velocity_to_number_scans_ratio * inverse_angular_velocity;
	number_of_scans_to_assemble_per_cloud_ = std::ceil(std::min(number_scans_linear_velocity, number_scans_angular_velocity));

	double linear_velocity_to_timeout_ratio = (max_timeout_seconds_for_cloud_assembly_ - min_timeout_seconds_for_cloud_assembly_) / max_linear_velocity_;
	double angular_velocity_to_timeout_ratio = (max_timeout_seconds_for_cloud_assembly_ - min_timeout_seconds_for_cloud_assembly_) / max_angular_velocity_;
	double timeout_linear_velocity = min_timeout_seconds_for_cloud_assembly_ + linear_velocity_to_timeout_ratio * inverse_linear_velocity;
	double timeout_angular_velocity = min_timeout_seconds_for_cloud_assembly_ + angular_velocity_to_timeout_ratio * inverse_angular_velocity;
	double timeout = std::min(timeout_linear_velocity, timeout_angular_velocity);
	timeout_for_cloud_assembly_ = rclcpp::Duration::from_nanoseconds(timeout * 1e9);

	RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 100, 
		"Laser scan assembly configuration: [ number_of_scans_to_assemble_per_cloud: %d ] | [ timeout_for_cloud_assembly: %.3fs ]",
		number_of_scans_to_assemble_per_cloud_, timeout);
}


void LaserScanToPointcloudAssembler::adjustAssemblyConfigurationFromTwist(const geometry_msgs::msg::Twist::SharedPtr twist) {
	adjustAssemblyConfiguration(twist->linear, twist->angular);
}


void LaserScanToPointcloudAssembler::adjustAssemblyConfigurationFromOdometry(const nav_msgs::msg::Odometry::SharedPtr odometry) {
	adjustAssemblyConfiguration(odometry->twist.twist.linear, odometry->twist.twist.angular);
}


void LaserScanToPointcloudAssembler::adjustAssemblyConfigurationFromIMU(const sensor_msgs::msg::Imu::SharedPtr imu) {
	if (imu_last_message_stamp_.seconds() > 0.1) {
		rclcpp::Time current_stamp(imu->header.stamp.sec, imu->header.stamp.nanosec);
		double imu_dt = (current_stamp - imu_last_message_stamp_).seconds();
		if (imu_dt > 0) {
			imu_linear_velocity.x += imu->linear_acceleration.x * imu_dt;
			imu_linear_velocity.y += imu->linear_acceleration.y * imu_dt;
			imu_linear_velocity.z += imu->linear_acceleration.z * imu_dt;
			adjustAssemblyConfiguration(imu_linear_velocity, imu->angular_velocity);
		}
	}

	imu_last_message_stamp_ = rclcpp::Time(imu->header.stamp.sec, imu->header.stamp.nanosec);
}

// NEW PX4 INTEGRATION METHODS
void LaserScanToPointcloudAssembler::adjustAssemblyConfigurationFromPX4Odometry(const px4_msgs::msg::VehicleOdometry::SharedPtr px4_odometry) {
	// Convert PX4 odometry to geometry_msgs format for existing algorithm
	geometry_msgs::msg::Vector3 linear_velocity;
	geometry_msgs::msg::Vector3 angular_velocity;
	
	// PX4 VehicleOdometry uses NED frame, velocity in m/s
	linear_velocity.x = px4_odometry->velocity[0];  // North
	linear_velocity.y = px4_odometry->velocity[1];  // East
	linear_velocity.z = px4_odometry->velocity[2];  // Down
	
	// Angular velocity in rad/s
	angular_velocity.x = px4_odometry->angular_velocity[0];
	angular_velocity.y = px4_odometry->angular_velocity[1];
	angular_velocity.z = px4_odometry->angular_velocity[2];
	
	// Use existing algorithm with PX4 data
	adjustAssemblyConfiguration(linear_velocity, angular_velocity);
	
	RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
		"PX4 motion compensation: linear_vel=[%.2f,%.2f,%.2f] m/s, angular_vel=[%.2f,%.2f,%.2f] rad/s",
		linear_velocity.x, linear_velocity.y, linear_velocity.z,
		angular_velocity.x, angular_velocity.y, angular_velocity.z);
}

void LaserScanToPointcloudAssembler::adjustAssemblyConfigurationFromPX4Attitude(const px4_msgs::msg::VehicleAttitude::SharedPtr px4_attitude) {
	// For now, we mainly use attitude for debugging/monitoring
	// The main motion compensation comes from odometry
	RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
		"PX4 attitude quaternion: [%.3f, %.3f, %.3f, %.3f]",
		px4_attitude->q[0], px4_attitude->q[1], px4_attitude->q[2], px4_attitude->q[3]);
}


// Dynamic parameter callback for real-time parameter updates
rcl_interfaces::msg::SetParametersResult LaserScanToPointcloudAssembler::parametersCallback(
	const std::vector<rclcpp::Parameter>& parameters) {
	
	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	result.reason = "Parameters updated successfully";
	
	for (const auto& param : parameters) {
		// Assembly parameters (most important for tuning)
		if (param.get_name() == "number_of_scans_to_assemble_per_cloud") {
			int new_value = param.as_int();
			if (new_value >= 1 && new_value <= 100) {
				number_of_scans_to_assemble_per_cloud_ = new_value;
				RCLCPP_INFO(node_->get_logger(), "Updated number_of_scans_to_assemble_per_cloud to: %d", new_value);
			} else {
				result.successful = false;
				result.reason = "number_of_scans_to_assemble_per_cloud must be between 1 and 100";
			}
		}
		else if (param.get_name() == "timeout_for_cloud_assembly") {
			double new_value = param.as_double();
			if (new_value > 0.0 && new_value <= 30.0) {
				timeout_for_cloud_assembly_ = rclcpp::Duration::from_nanoseconds(new_value * 1e9);
				RCLCPP_INFO(node_->get_logger(), "Updated timeout_for_cloud_assembly to: %.2f seconds", new_value);
			} else {
				result.successful = false;
				result.reason = "timeout_for_cloud_assembly must be between 0.1 and 30.0 seconds";
			}
		}
		
		// Dynamic range parameters
		else if (param.get_name() == "min_number_of_scans_to_assemble_per_cloud") {
			int new_value = param.as_int();
			if (new_value >= 1 && new_value <= max_number_of_scans_to_assemble_per_cloud_) {
				min_number_of_scans_to_assemble_per_cloud_ = new_value;
				RCLCPP_INFO(node_->get_logger(), "Updated min_number_of_scans_to_assemble_per_cloud to: %d", new_value);
			} else {
				result.successful = false;
				result.reason = "min_number_of_scans must be >= 1 and <= max_number_of_scans";
			}
		}
		else if (param.get_name() == "max_number_of_scans_to_assemble_per_cloud") {
			int new_value = param.as_int();
			if (new_value >= min_number_of_scans_to_assemble_per_cloud_ && new_value <= 200) {
				max_number_of_scans_to_assemble_per_cloud_ = new_value;
				RCLCPP_INFO(node_->get_logger(), "Updated max_number_of_scans_to_assemble_per_cloud to: %d", new_value);
			} else {
				result.successful = false;
				result.reason = "max_number_of_scans must be >= min_number_of_scans and <= 200";
			}
		}
		
		// Motion velocity limits
		else if (param.get_name() == "max_linear_velocity") {
			double new_value = param.as_double();
			if (new_value > 0.0 && new_value <= 50.0) {
				max_linear_velocity_ = new_value;
				RCLCPP_INFO(node_->get_logger(), "Updated max_linear_velocity to: %.2f m/s", new_value);
			} else {
				result.successful = false;
				result.reason = "max_linear_velocity must be between 0.1 and 50.0 m/s";
			}
		}
		else if (param.get_name() == "max_angular_velocity") {
			double new_value = param.as_double();
			if (new_value > 0.0 && new_value <= 10.0) {
				max_angular_velocity_ = new_value;
				RCLCPP_INFO(node_->get_logger(), "Updated max_angular_velocity to: %.2f rad/s", new_value);
			} else {
				result.successful = false;
				result.reason = "max_angular_velocity must be between 0.1 and 10.0 rad/s";
			}
		}
		
		// Timeout range parameters
		else if (param.get_name() == "min_timeout_seconds_for_cloud_assembly") {
			double new_value = param.as_double();
			if (new_value > 0.0 && new_value <= max_timeout_seconds_for_cloud_assembly_) {
				min_timeout_seconds_for_cloud_assembly_ = new_value;
				RCLCPP_INFO(node_->get_logger(), "Updated min_timeout_seconds_for_cloud_assembly to: %.2f", new_value);
			} else {
				result.successful = false;
				result.reason = "min_timeout must be > 0 and <= max_timeout";
			}
		}
		else if (param.get_name() == "max_timeout_seconds_for_cloud_assembly") {
			double new_value = param.as_double();
			if (new_value >= min_timeout_seconds_for_cloud_assembly_ && new_value <= 60.0) {
				max_timeout_seconds_for_cloud_assembly_ = new_value;
				RCLCPP_INFO(node_->get_logger(), "Updated max_timeout_seconds_for_cloud_assembly to: %.2f", new_value);
			} else {
				result.successful = false;
				result.reason = "max_timeout must be >= min_timeout and <= 60.0 seconds";
			}
		}
		
		// Core algorithm parameters (require more validation)
		else if (param.get_name() == "number_of_tf_queries_for_spherical_interpolation") {
			int new_value = param.as_int();
			if (new_value >= 1 && new_value <= 32) {
				laserscan_to_pointcloud_.setNumberOfTfQueriesForSphericalInterpolation(new_value);
				RCLCPP_INFO(node_->get_logger(), "Updated TF queries for spherical interpolation to: %d", new_value);
			} else {
				result.successful = false;
				result.reason = "number_of_tf_queries must be between 1 and 32";
			}
		}
		else if (param.get_name() == "tf_lookup_timeout") {
			double new_value = param.as_double();
			if (new_value > 0.0 && new_value <= 5.0) {
				laserscan_to_pointcloud_.setTFLookupTimeout(new_value);
				RCLCPP_INFO(node_->get_logger(), "Updated TF lookup timeout to: %.3f seconds", new_value);
			} else {
				result.successful = false;
				result.reason = "tf_lookup_timeout must be between 0.01 and 5.0 seconds";
			}
		}
		
		// Range cutoff parameters
		else if (param.get_name() == "min_range_cutoff_percentage_offset") {
			double new_value = param.as_double();
			if (new_value >= 0.5 && new_value <= 2.0) {
				laserscan_to_pointcloud_.setMinRangeCutoffPercentageOffset(new_value);
				RCLCPP_INFO(node_->get_logger(), "Updated min_range_cutoff_percentage_offset to: %.3f", new_value);
			} else {
				result.successful = false;
				result.reason = "min_range_cutoff_percentage_offset must be between 0.5 and 2.0";
			}
		}
		else if (param.get_name() == "max_range_cutoff_percentage_offset") {
			double new_value = param.as_double();
			if (new_value >= 0.5 && new_value <= 1.5) {
				laserscan_to_pointcloud_.setMaxRangeCutoffPercentageOffset(new_value);
				RCLCPP_INFO(node_->get_logger(), "Updated max_range_cutoff_percentage_offset to: %.3f", new_value);
			} else {
				result.successful = false;
				result.reason = "max_range_cutoff_percentage_offset must be between 0.5 and 1.5";
			}
		}
		
		// Intensity parameter
		else if (param.get_name() == "include_laser_intensity") {
			bool new_value = param.as_bool();
			include_laser_intensity_ = new_value;
			laserscan_to_pointcloud_.setIncludeLaserIntensity(new_value);
			RCLCPP_INFO(node_->get_logger(), "Updated include_laser_intensity to: %s", new_value ? "true" : "false");
		}
		
		// Invalid measurements parameter
		else if (param.get_name() == "remove_invalid_measurements") {
			bool new_value = param.as_bool();
			laserscan_to_pointcloud_.setRemoveInvalidMeasurements(new_value);
			RCLCPP_INFO(node_->get_logger(), "Updated remove_invalid_measurements to: %s", new_value ? "true" : "false");
		}
		
		// Frame parameters (require restart warning)
		else if (param.get_name() == "target_frame" || param.get_name() == "laser_frame" ||
				 param.get_name() == "motion_estimation_source_frame_id" || param.get_name() == "motion_estimation_target_frame_id") {
			RCLCPP_WARN(node_->get_logger(), "Frame parameter '%s' changed - node restart required for full effect", param.get_name().c_str());
			result.successful = true; // Allow change but warn
		}
		
		// Topic parameters (require restart)
		else if (param.get_name() == "laser_scan_topics" || param.get_name() == "pointcloud_publish_topic" ||
				 param.get_name() == "px4_odometry_topic" || param.get_name() == "px4_attitude_topic") {
			RCLCPP_WARN(node_->get_logger(), "Topic parameter '%s' changed - node restart required", param.get_name().c_str());
			result.successful = true; // Allow change but warn
		}
		
		else {
			RCLCPP_DEBUG(node_->get_logger(), "Parameter '%s' not handled by dynamic callback", param.get_name().c_str());
		}
		
		// Break on first failure
		if (!result.successful) {
			break;
		}
	}
	
	return result;
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </LaserScanToPointcloudAssembler-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>   ==========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================



} /* namespace laserscan_to_pointcloud */
