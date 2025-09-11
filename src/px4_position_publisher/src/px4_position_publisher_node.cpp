#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/common/context.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_position_publisher/msg/full_state.hpp>
#include <array>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <px4_ros_com/frame_transforms.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class PX4PositionPublisherNode : public rclcpp::Node
{
public:
    PX4PositionPublisherNode() : Node("px4_position_publisher")
    {
        // Create context with the correct namespace
        // The namespace will be automatically prepended by ROS
        context_ = std::make_unique<px4_ros2::Context>(*this, "");
        
        // Create local position subscription
        //local_position_ = std::make_unique<px4_ros2::OdometryLocalPosition>(*context_, false);
        
        // Create publisher for full state message
        state_pub_ = create_publisher<px4_position_publisher::msg::FullState>(
            "px4_full_state", 10);
            

        // Initialize TF broadcaster and transform message
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        transform_stamped_ = std::make_shared<geometry_msgs::msg::TransformStamped>();
        transform_stamped_->header.frame_id = "local_origin";
        transform_stamped_->child_frame_id = "px4_body";

        // Subscribe to vehicle odometry for transform
        // auto qos = rclcpp::QoS(15)  // Increased buffer for better TF interpolation
        //     .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        //     .durability(rclcpp::DurabilityPolicy::Volatile)
        //     .history(rclcpp::HistoryPolicy::KeepLast);


        //note namesoace from lauch!!
        
        // odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        //       "/px4_0/fmu/out/vehicle_odometry", qos,
        //     std::bind(&PX4PositionPublisherNode::vehicleOdometryCallback, this, std::placeholders::_1));


        // local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        //       "/px4_0/fmu/out/vehicle_local_position", qos,
        //     std::bind(&PX4PositionPublisherNode::vehicleLocalPositionCallback, this, std::placeholders::_1));


        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // Initialize message filter subscribers with PX4 QoS
        odom_sub_.subscribe(this, "/px4_0/fmu/out/vehicle_odometry", qos.get_rmw_qos_profile());
        local_pos_sub_.subscribe(this, "/px4_0/fmu/out/vehicle_local_position_v1", qos.get_rmw_qos_profile());
        
        // Create synchronizer
        sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), odom_sub_, local_pos_sub_);
        sync_->registerCallback(
            std::bind(&PX4PositionPublisherNode::syncCallback, this, 
                     std::placeholders::_1, std::placeholders::_2)
        );

        


        
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to vehicle odometry topic '%s' to publish transform from local origin to px4 body", 
            odom_sub_.getTopic().c_str());
        

        RCLCPP_INFO(this->get_logger(), "PX4 Position Publisher node started!");
    }

    void syncCallback(const px4_msgs::msg::VehicleOdometry::ConstSharedPtr& odometry_msg, 
                     const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr& msg_local_pos)
    {
        // // Print comprehensive PX4 data in one message
        // RCLCPP_INFO(this->get_logger(),
        //     "\n=== PX4 Vehicle Data ===\n"
        //     "Validity: XY=%s Z=%s VXY=%s VZ=%s\n"
        //     "Position (NED): [%.3f, %.3f, %.3f] m\n"
        //     "Velocity (NED): [%.3f, %.3f, %.3f] m/s (Z_deriv: %.3f)\n"
        //     "Acceleration (NED): [%.3f, %.3f, %.3f] m/s²\n"
        //     "Angular Velocity (NED): [%.3f, %.3f, %.3f] rad/s\n"
        //     "Reset Counters: XY=%d Z=%d VXY=%d VZ=%d\n"
        //     "Deltas: XY=[%.3f, %.3f] Z=%.3f VXY=[%.3f, %.3f] VZ=%.3f",
        //     msg_local_pos->xy_valid ? "✓" : "✗",
        //     msg_local_pos->z_valid ? "✓" : "✗", 
        //     msg_local_pos->v_xy_valid ? "✓" : "✗",
        //     msg_local_pos->v_z_valid ? "✓" : "✗",
        //     msg_local_pos->x, msg_local_pos->y, msg_local_pos->z,
        //     msg_local_pos->vx, msg_local_pos->vy, msg_local_pos->vz, msg_local_pos->z_deriv,
        //     msg_local_pos->ax, msg_local_pos->ay, msg_local_pos->az,
        //     odometry_msg->angular_velocity[0], odometry_msg->angular_velocity[1], odometry_msg->angular_velocity[2],
        //     msg_local_pos->xy_reset_counter, msg_local_pos->z_reset_counter,
        //     msg_local_pos->vxy_reset_counter, msg_local_pos->vz_reset_counter,
        //     msg_local_pos->delta_xy[0], msg_local_pos->delta_xy[1], msg_local_pos->delta_z,
        //     msg_local_pos->delta_vxy[0], msg_local_pos->delta_vxy[1], msg_local_pos->delta_vz);

        // Create full state message
        auto full_state_msg = px4_position_publisher::msg::FullState();
        
        // Header
        full_state_msg.header.stamp = this->get_clock()->now();
        full_state_msg.header.frame_id = "local_origin";
        //full_state_msg.child_frame_id = "px4_body";
        // Convert position from NED to ENU
        Eigen::Vector3d pos_ned(odometry_msg->position[0], odometry_msg->position[1], odometry_msg->position[2]);
        Eigen::Vector3d pos_enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(pos_ned);
        
        // Convert velocity from NED to ENU
        Eigen::Vector3d vel_ned(msg_local_pos->vx, msg_local_pos->vy, msg_local_pos->vz);
        Eigen::Vector3d vel_enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(vel_ned);
        
        // Convert acceleration from NED to ENU
        Eigen::Vector3d accel_ned(msg_local_pos->ax, msg_local_pos->ay, msg_local_pos->az);
        Eigen::Vector3d accel_enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(accel_ned);
        
        // Convert quaternion from NED to ENU
        Eigen::Quaterniond q_ned(odometry_msg->q[0], odometry_msg->q[1], odometry_msg->q[2], odometry_msg->q[3]);
        Eigen::Quaterniond q_enu = px4_ros_com::frame_transforms::ned_to_enu_orientation(q_ned);
        
        // Convert angular velocity from NED to ENU
        Eigen::Vector3d angular_ned(odometry_msg->angular_velocity[0], odometry_msg->angular_velocity[1], odometry_msg->angular_velocity[2]);
        Eigen::Vector3d angular_enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(angular_ned);
        
        // Fill position
        full_state_msg.position.x = pos_enu.x();
        full_state_msg.position.y = pos_enu.y();
        full_state_msg.position.z = pos_enu.z();
        
        // Fill velocity
        full_state_msg.velocity.x = vel_enu.x();
        full_state_msg.velocity.y = vel_enu.y();
        full_state_msg.velocity.z = vel_enu.z();
        
        // Fill acceleration
        full_state_msg.acceleration.x = accel_enu.x();
        full_state_msg.acceleration.y = accel_enu.y();
        full_state_msg.acceleration.z = accel_enu.z();
        
        // Fill orientation
        full_state_msg.orientation.w = q_enu.w();
        full_state_msg.orientation.x = q_enu.x();
        full_state_msg.orientation.y = q_enu.y();
        full_state_msg.orientation.z = q_enu.z();
        
        // Fill angular velocity
        full_state_msg.angular_velocity.x = angular_enu.x();
        full_state_msg.angular_velocity.y = angular_enu.y();
        full_state_msg.angular_velocity.z = angular_enu.z();
        
        // Fill covariance matrices from PX4 uncertainty estimates
        // Position covariance (6x6 matrix in row-major order)
        std::fill(full_state_msg.pose_covariance.begin(), full_state_msg.pose_covariance.end(), 0.0);
        full_state_msg.pose_covariance[0] = msg_local_pos->eph * msg_local_pos->eph;   // x variance
        full_state_msg.pose_covariance[7] = msg_local_pos->eph * msg_local_pos->eph;   // y variance  
        full_state_msg.pose_covariance[14] = msg_local_pos->epv * msg_local_pos->epv;  // z variance
        
        // Velocity covariance (6x6 matrix in row-major order)
        std::fill(full_state_msg.twist_covariance.begin(), full_state_msg.twist_covariance.end(), 0.0);
        full_state_msg.twist_covariance[0] = msg_local_pos->evh * msg_local_pos->evh;  // vx variance
        full_state_msg.twist_covariance[7] = msg_local_pos->evh * msg_local_pos->evh;  // vy variance
        full_state_msg.twist_covariance[14] = msg_local_pos->evv * msg_local_pos->evv; // vz variance
        
        // Publish the full state message
        state_pub_->publish(full_state_msg);

        // Log final converted values nicely
        RCLCPP_INFO(this->get_logger(),
            "\n=== PX4 Full State (ENU) ===\n"
            "Position: [%.3f, %.3f, %.3f] m\n"
            "Velocity: [%.3f, %.3f, %.3f] m/s\n"
            "Acceleration: [%.3f, %.3f, %.3f] m/s²\n"
            "Orientation: [%.3f, %.3f, %.3f, %.3f] (w,x,y,z)\n"
            "Angular Vel: [%.3f, %.3f, %.3f] rad/s\n"
            "Covariance: pos=[%.3f,%.3f,%.3f] vel=[%.3f,%.3f,%.3f]",
            full_state_msg.position.x, full_state_msg.position.y, full_state_msg.position.z,
            full_state_msg.velocity.x, full_state_msg.velocity.y, full_state_msg.velocity.z,
            full_state_msg.acceleration.x, full_state_msg.acceleration.y, full_state_msg.acceleration.z,
            full_state_msg.orientation.w, full_state_msg.orientation.x, 
            full_state_msg.orientation.y, full_state_msg.orientation.z,
            full_state_msg.angular_velocity.x, full_state_msg.angular_velocity.y, full_state_msg.angular_velocity.z,
            full_state_msg.pose_covariance[0], full_state_msg.pose_covariance[7], full_state_msg.pose_covariance[14],
            full_state_msg.twist_covariance[0], full_state_msg.twist_covariance[7], full_state_msg.twist_covariance[14]);

        // Update transform
        updateTransform(*odometry_msg);
    }

    void updateTransform(const px4_msgs::msg::VehicleOdometry& msg)
    {
        // Update transform timestamp
        transform_stamped_->header.stamp = this->now();
        
        // Convert PX4 NED position to ROS ENU
        Eigen::Vector3d pos_ned(msg.position[0], msg.position[1], msg.position[2]);
        Eigen::Vector3d pos_enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(pos_ned);
        
        transform_stamped_->transform.translation.x = pos_enu.x();
        transform_stamped_->transform.translation.y = pos_enu.y();
        transform_stamped_->transform.translation.z = pos_enu.z();
        
        // Convert PX4 NED quaternion to ROS ENU
        // PX4 quaternion order is [w, x, y, z]
        Eigen::Quaterniond q_ned(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
        Eigen::Quaterniond q_enu = px4_ros_com::frame_transforms::ned_to_enu_orientation(q_ned);
        
        transform_stamped_->transform.rotation.x = q_enu.x();
        transform_stamped_->transform.rotation.y = q_enu.y();
        transform_stamped_->transform.rotation.z = q_enu.z();
        transform_stamped_->transform.rotation.w = q_enu.w();
        
        // Broadcast transform
        RCLCPP_INFO(this->get_logger(), "Broadcasting transform from local origin to px4 body");
        tf_broadcaster_->sendTransform(*transform_stamped_);
    }

    void vehicleOdometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received vehicle odometry message");
        // Update transform timestamp
        transform_stamped_->header.stamp = this->now();
        
        // Convert PX4 NED position to ROS ENU
        Eigen::Vector3d pos_ned(msg->position[0], msg->position[1], msg->position[2]);
        Eigen::Vector3d pos_enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(pos_ned);
        
        transform_stamped_->transform.translation.x = pos_enu.x();
        transform_stamped_->transform.translation.y = pos_enu.y();
        transform_stamped_->transform.translation.z = pos_enu.z();
        
        // Convert PX4 NED quaternion to ROS ENU
        // PX4 quaternion order is [w, x, y, z]
        Eigen::Quaterniond q_ned(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
        Eigen::Quaterniond q_enu = px4_ros_com::frame_transforms::ned_to_enu_orientation(q_ned);
        
        transform_stamped_->transform.rotation.x = q_enu.x();
        transform_stamped_->transform.rotation.y = q_enu.y();
        transform_stamped_->transform.rotation.z = q_enu.z();
        transform_stamped_->transform.rotation.w = q_enu.w();
        
        // Broadcast transform
        RCLCPP_INFO(this->get_logger(), "Broadcasting transform from local origin to px4 body");
        tf_broadcaster_->sendTransform(*transform_stamped_);
    }

private:
    std::unique_ptr<px4_ros2::Context> context_;

    std::unique_ptr<px4_ros2::OdometryLocalPosition> local_position_;
    rclcpp::Publisher<px4_position_publisher::msg::FullState>::SharedPtr state_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<geometry_msgs::msg::TransformStamped> transform_stamped_;
    // Message filter subscribers
    message_filters::Subscriber<px4_msgs::msg::VehicleOdometry> odom_sub_;
    message_filters::Subscriber<px4_msgs::msg::VehicleLocalPosition> local_pos_sub_;

    // Synchronizer
    typedef message_filters::sync_policies::ApproximateTime<
        px4_msgs::msg::VehicleOdometry, 
        px4_msgs::msg::VehicleLocalPosition
    > SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    std::shared_ptr<Synchronizer> sync_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4PositionPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
