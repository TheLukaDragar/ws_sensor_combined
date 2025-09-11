/****************************************************************************
 * Copyright (c) 2024 PX4 TF Publisher.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "rclcpp/rclcpp.hpp"

#include <px4_tf_publisher.hpp>


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4TFPublisher>());
  rclcpp::shutdown();
  return 0;
}