# Collision Prevention Node

A ROS2 node that converts `sensor_msgs/LaserScan` messages to `px4_msgs/ObstacleDistance` messages for PX4's collision prevention system.

## Overview

This node subscribes to LaserScan data from a 2D LiDAR sensor (like the LD19) and converts it to the ObstacleDistance format expected by PX4's collision prevention system. The conversion follows PX4's standard practices:

- Downsamples laser data into 5-degree sectors (72 sectors total for 360°)
- Converts from meters to centimeters
- Handles coordinate frame conversion (FLU to FRD)
- Averages multiple laser readings per sector

## Usage

### Build the package

```bash
cd ~/ws_sensor_combined
colcon build --packages-select collision_prevention
source install/setup.bash
```

### Launch the node

```bash
# Basic launch
ros2 launch collision_prevention collision_prevention.launch.py

# With custom parameters
ros2 launch collision_prevention collision_prevention.launch.py \
  input_topic:=/your_lidar/scan \
  output_topic:=/fmu/in/obstacle_distance
```

### Run directly

```bash
ros2 run collision_prevention collision_prevention_node
```

## Configuration

Configure the node using the parameters in `config/collision_prevention.yaml`:

- `input_topic`: LaserScan input topic (default: `/ld19/scan`)
- `output_topic`: ObstacleDistance output topic (default: `/fmu/in/obstacle_distance`)
- `sector_size_deg`: Sector size in degrees (default: 5, matches PX4 standard)

## Topics

### Subscribed Topics

- `input_topic` (`sensor_msgs/LaserScan`): 2D laser scan data from LiDAR sensor

### Published Topics

- `output_topic` (`px4_msgs/ObstacleDistance`): Obstacle distance data for PX4 collision prevention

## PX4 Integration

To use with PX4 collision prevention:

1. Ensure PX4 parameters are set:
   - `CP_DIST`: Minimum allowed distance (e.g., 1.5m)
   - `CP_DELAY`: Sensor delay (e.g., 0.1s) 
   - `CP_GUIDE_ANG`: Guidance angle (e.g., 30°)
   - `MPC_POS_MODE`: Set to "Acceleration based"

2. This node publishes to `/fmu/in/obstacle_distance` which PX4 uses for collision prevention

## Algorithm

The conversion algorithm:

1. Groups laser scan rays into 5-degree sectors
2. Averages valid readings within each sector
3. Maps sectors from sensor frame to PX4's body frame (FRD)
4. Converts distances from meters to centimeters
5. Publishes as ObstacleDistance message with 72 sectors

This follows the same pattern as PX4's internal Gazebo bridge implementation.