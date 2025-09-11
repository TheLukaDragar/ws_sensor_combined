# PX4 Sensor Integration & Autonomous Navigation Stack

A comprehensive ROS 2 workspace containing packages for autonomous drone navigation, sensor processing, collision avoidance, and mapping using PX4 autopilot.

## Installation

### Clone with Submodules

This repository uses Git submodules for PX4 dependencies. To clone the repository with all its submodules, use:

```bash
# Clone the main repository
git clone https://github.com/your_username/ws_sensor_combined.git

# Enter the repository
cd ws_sensor_combined

# Initialize and fetch submodules
git submodule update --init --recursive
```

Alternatively, you can clone with submodules in one command:
```bash
git clone --recursive https://github.com/your_username/ws_sensor_combined.git
```

### Updating Submodules

To update the submodules to their latest versions:
```bash
git submodule update --remote
```

### Submodule Dependencies
This workspace includes the following submodules:
- [px4_msgs](https://github.com/TheLukaDragar/px4_msgs) - ROS 2 message definitions for PX4
- [px4-ros2-interface-lib](https://github.com/TheLukaDragar/px4-ros2-interface-lib) - PX4 ROS 2 interface library

## Building the Workspace

After cloning:

```bash
# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Node Overview

### PX4 Position Publisher Node
- **Main Node**: `px4_position_publisher_node`
- **Purpose**: Converts PX4 state data from NED to ENU frame
- **Key Features**:
  - Publishes complete state (position, velocity, acceleration)
  - Broadcasts TF transforms
  - Handles coordinate frame conversion
  - Synchronized odometry processing

### LaserScan to PointCloud Node
- **Main Node**: `px4_pointcloud_assembler`
- **Purpose**: Creates 3D point clouds from 2D laser scans
- **Key Features**:
  - Spherical linear interpolation
  - PX4 motion compensation
  - Dynamic scan assembly based on motion
  - Real-time performance optimization

### Collision Prevention Node
- **Main Node**: `collision_prevention_node`
- **Purpose**: Converts LaserScan to PX4 ObstacleDistance format
- **Key Features**:
  - 5-degree sector downsampling
  - Coordinate frame conversion (FLU to FRD)
  - Direct PX4 collision prevention integration
  - Real-time obstacle detection

### Collision-Aware Flight Mode Node
- **Main Node**: `collision_autonomous_mode_node`
- **Purpose**: Autonomous navigation with collision avoidance
- **Key Features**:
  - Waypoint navigation
  - Real-time obstacle avoidance
  - Path planning
  - Safety features and fail-safes
  - Integration with PX4 flight modes

## System Architecture

The nodes work together in the following way:
1. `px4_position_publisher_node` provides accurate state estimation
2. `px4_pointcloud_assembler` creates 3D environment representation
3. `collision_prevention_node` processes sensor data for obstacle detection
4. `collision_autonomous_mode_node` uses this information for safe navigation

## Prerequisites

- ROS 2 (tested with Humble)
- PX4 Autopilot
- OpenVDB (for mapping)
- Eigen3
- Additional dependencies listed in individual packages

## Usage

Each package contains its own launch files and configuration. See individual package READMEs for detailed usage instructions.

Common launch scenarios:

1. Basic PX4 integration:
```bash
ros2 launch px4_position_publisher px4_position_publisher.launch.py
```

2. Full autonomous stack:
```bash
# Launch nodes in order
ros2 launch px4_position_publisher px4_position_publisher.launch.py
ros2 launch laserscan_to_pointcloud cloud_px4_real.launch.py
ros2 launch collision_prevention collision_prevention.launch.py
ros2 launch collision_flightmode collision_autonomous_mode.launch.py
```

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.