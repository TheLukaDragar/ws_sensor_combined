#pragma once

/**\file tf_rosmsg_eigen_conversions.h
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <macros>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </macros>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes

// ROS includes
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>

// external libs includes
#include <Eigen/Core>
#include <Eigen/Geometry>

// project includes

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace laserscan_to_pointcloud {

// ##############################################################################   tf2_convertions   #############################################################################
namespace tf_rosmsg_eigen_conversions {

void transformMsgToTF2(const geometry_msgs::msg::Transform& msg, tf2::Transform& tf2);
void transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::msg::Transform& msg);

void transformMsgToTF2(const geometry_msgs::msg::Pose& msg, tf2::Transform& tf2);
void transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::msg::Pose& msg);

void transformMsgToTF2(const geometry_msgs::msg::Quaternion& msg, tf2::Quaternion& tf2);
void transformTF2ToMsg(const tf2::Quaternion& tf2, geometry_msgs::msg::Quaternion& msg);

void transformMsgToTF2(const geometry_msgs::msg::Vector3& msg, tf2::Vector3& tf2);
void transformTF2ToMsg(const tf2::Vector3& tf2, geometry_msgs::msg::Vector3& msg);

void transformMsgToTF2(const geometry_msgs::msg::Point& msg, tf2::Vector3& tf2);
void transformTF2ToMsg(const tf2::Vector3& tf2, geometry_msgs::msg::Point& msg);

void transformMatrixToTF2(const Eigen::Matrix4f& matrix, tf2::Transform& tf2);
void transformMatrixToMsg(const Eigen::Matrix4f& matrix, geometry_msgs::msg::Pose& msg);

template <typename Scalar>
Eigen::Transform<Scalar, 3, Eigen::Affine> transformTF2ToTransform(const tf2::Transform& tf2) {
	Eigen::Translation<Scalar, 3> translation(tf2.getOrigin().getX(), tf2.getOrigin().getY(), tf2.getOrigin().getZ());
	Eigen::Quaternion<Scalar> rotation(tf2.getRotation().getW(), tf2.getRotation().getX(), tf2.getRotation().getY(), tf2.getRotation().getZ());
	return Eigen::Transform<Scalar, 3, Eigen::Affine>(translation * rotation);
}

} /* namespace tf_rosmsg_eigen_conversions */
} /* namespace laserscan_to_pointcloud */
