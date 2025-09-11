# PX4 Position Publisher

A ROS 2 package that publishes PX4 drone state information in ROS-friendly ENU (East-North-Up) coordinate frame. This package converts PX4's native NED (North-East-Down) frame data to the ROS standard ENU frame and provides a comprehensive state publisher including position, velocity, acceleration, orientation, and their uncertainties.

## Features

- Converts PX4 state data from NED to ENU coordinate frame
- Publishes full state information including:
  - Position (ENU)
  - Velocity (ENU)
  - Acceleration (ENU)
  - Orientation (quaternion in ENU)
  - Angular velocity (ENU)
  - Pose and twist covariance matrices
- Broadcasts TF transforms from local_origin to px4_body
- Synchronized processing of odometry and local position data
- Comprehensive state logging for debugging

## Prerequisites

- ROS 2 (tested with Humble)
- PX4 Autopilot
- px4_ros_com package
- px4_msgs package
- Eigen3

## Installation

1. Clone this repository into your ROS 2 workspace's src directory:
```bash
cd ~/your_ws/src
git clone https://github.com/your_username/px4_position_publisher.git
```

2. Install dependencies:
```bash
cd ~/your_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --packages-select px4_position_publisher
```

4. Source the workspace:
```bash
source install/setup.bash
```

## Usage

1. Launch the position publisher node:
```bash
ros2 launch px4_position_publisher px4_position_publisher.launch.py
```

2. The node will publish the following topics:
- `/px4_full_state` - Complete state information in ENU frame
- `/tf` - Transform from local_origin to px4_body

## Topics

### Subscribed Topics
- `/px4_0/fmu/out/vehicle_odometry` (px4_msgs/msg/VehicleOdometry)
- `/px4_0/fmu/out/vehicle_local_position_v1` (px4_msgs/msg/VehicleLocalPosition)

### Published Topics
- `/px4_full_state` (px4_position_publisher/msg/FullState)
- `/tf` (tf2_msgs/msg/TFMessage)

## Custom Messages

### FullState
Contains the complete state of the PX4 vehicle in ENU frame:
- Header
- Position (x, y, z)
- Velocity (x, y, z)
- Acceleration (x, y, z)
- Orientation (quaternion)
- Angular velocity (x, y, z)
- Pose covariance matrix
- Twist covariance matrix

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Authors

- Your Name (<your_email>)
