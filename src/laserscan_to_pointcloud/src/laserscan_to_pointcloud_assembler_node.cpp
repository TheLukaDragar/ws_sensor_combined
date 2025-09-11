/**\file laserscan_to_pointcloud_node.cpp
 * \brief ROS2 node for LaserScan to PointCloud conversion with PX4 integration
 *
 * @version 2.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <rclcpp/rclcpp.hpp>
#include <laserscan_to_pointcloud/laserscan_to_pointcloud_assembler.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// ###################################################################################   <main>   ##############################################################################
int main(int argc, char** argv) {
	rclcpp::init(argc, argv);

	auto node = std::make_shared<rclcpp::Node>("laserscan_to_pointcloud_assembler");
	
	auto laserscan_to_pointcloud_assembler = std::make_shared<laserscan_to_pointcloud::LaserScanToPointcloudAssembler>(node);
	laserscan_to_pointcloud_assembler->startAssemblingLaserScans();

	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
// ###################################################################################   </main>   #############################################################################
