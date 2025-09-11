/**\file tf_collector.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <laserscan_to_pointcloud/tf_collector.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace laserscan_to_pointcloud {
// =============================================================================  <public-section>   ===========================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
TFCollector::TFCollector(std::shared_ptr<rclcpp::Node> node, rclcpp::Duration buffer_duration) :
		node_(node),
		tf2_buffer_(node->get_clock(), tf2::durationFromSec(buffer_duration.seconds())),
		tf2_transform_listener_(tf2_buffer_, node, true) {  // true = use_static_transforms
}

TFCollector::~TFCollector() {
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <TFCollector-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
bool TFCollector::collectTFs(const std::string& target_frame, const std::string& source_frame, const rclcpp::Time& start_time, const rclcpp::Time& endtime, size_t number_tfs, std::vector<tf2::Transform>& collected_tfs_out, const rclcpp::Duration& tf_timeout) {
	collected_tfs_out.clear();

	rclcpp::Time current_tf_time = start_time;
	rclcpp::Duration next_tf_time_increment = rclcpp::Duration::from_nanoseconds((endtime - start_time).nanoseconds() / (number_tfs - 1));
	for (size_t tf_number = 0; tf_number < number_tfs; ++tf_number) {
		tf2::Transform tf2;
		if (lookForTransform(tf2, target_frame, source_frame, current_tf_time, tf_timeout)) {
			collected_tfs_out.push_back(tf2);
		}
		current_tf_time += next_tf_time_increment;
	}

	return !collected_tfs_out.empty();
}

bool TFCollector::lookForLatestTransform(tf2::Transform& tf2_transformOut, const std::string& target_frame, const std::string& source_frame, const rclcpp::Duration& timeout, size_t number_of_queries) {
	rclcpp::Time start_time = node_->get_clock()->now();
	rclcpp::Time end_time = start_time + timeout;
	rclcpp::Duration lookup_timeout = rclcpp::Duration::from_nanoseconds(timeout.nanoseconds() / ((double) number_of_queries));

	while (node_->get_clock()->now() < end_time) {
		if (lookForTransform(tf2_transformOut, target_frame, source_frame, rclcpp::Time(0), lookup_timeout)) {
			return true;
		}
	}

	return false;
}

bool TFCollector::lookForTransform(tf2::Vector3& translation_out, tf2::Quaternion& rotation_out, const std::string& target_frame, const std::string& source_frame, const rclcpp::Time& time, const rclcpp::Duration& timeout) {
	try {
		std::string source_frame_stripped = source_frame;
		std::string target_frame_stripped = target_frame;
		stripSlash(source_frame_stripped);
		stripSlash(target_frame_stripped);
		geometry_msgs::msg::TransformStamped tf = tf2_buffer_.lookupTransform(target_frame_stripped, source_frame_stripped, time, timeout);
		tf_rosmsg_eigen_conversions::transformMsgToTF2(tf.transform.translation, translation_out);
		tf_rosmsg_eigen_conversions::transformMsgToTF2(tf.transform.rotation, rotation_out);
		return true;
	} catch (const tf2::TransformException& ex) {
		RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to lookup transform from " << source_frame << " to " << target_frame << ": " << ex.what());
		return false;
	}
}

bool TFCollector::lookForTransform(tf2::Transform& tf2_transform_out, const std::string& target_frame, const std::string& source_frame, const rclcpp::Time& time, const rclcpp::Duration& timeout) {
	try {
		std::string source_frame_stripped = source_frame;
		std::string target_frame_stripped = target_frame;
		stripSlash(source_frame_stripped);
		stripSlash(target_frame_stripped);
		geometry_msgs::msg::TransformStamped tf = tf2_buffer_.lookupTransform(target_frame_stripped, source_frame_stripped, time, timeout);
		tf_rosmsg_eigen_conversions::transformMsgToTF2(tf.transform, tf2_transform_out);
		return true;
	} catch (const tf2::TransformException& ex) {
		RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to lookup transform from " << source_frame << " to " << target_frame << ": " << ex.what());
		return false;
	}
}

bool TFCollector::lookForTransform(tf2::Transform& tf2_transform_out, const std::string& target_frame, const rclcpp::Time& target_time, const std::string& source_frame, const rclcpp::Time& source_time, const std::string& fixed_frame, const rclcpp::Duration& timeout) {
	try {
		std::string source_frame_stripped = source_frame;
		std::string target_frame_stripped = target_frame;
		stripSlash(source_frame_stripped);
		stripSlash(target_frame_stripped);
		geometry_msgs::msg::TransformStamped tf = tf2_buffer_.lookupTransform(target_frame_stripped, target_time, source_frame_stripped, source_time, fixed_frame, timeout);
		tf_rosmsg_eigen_conversions::transformMsgToTF2(tf.transform, tf2_transform_out);
		return true;
	} catch (const tf2::TransformException& ex) {
		RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to lookup transform from " << source_frame << " to " << target_frame << " between times: " << ex.what());
		return false;
	}
}

bool TFCollector::startsWithSlash(const std::string& frame_id) {
	if (frame_id.size() > 0) if (frame_id[0] == '/') return true;
	return false;
}

void TFCollector::stripSlash(std::string& frame_id) {
	if (startsWithSlash(frame_id)) frame_id.erase(0, 1);
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </TFCollector-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>   ==========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================
} /* namespace laserscan_to_pointcloud */

