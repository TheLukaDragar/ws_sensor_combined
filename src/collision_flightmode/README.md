# Collision-Aware Autonomous Flight Mode

A collision-aware autonomous flight mode for PX4 using the PX4 ROS2 Interface Library. This mode enables autonomous waypoint navigation with real-time collision avoidance.

## Overview

This package provides:
- **Autonomous waypoint navigation** with collision avoidance
- **Integration with PX4 ROS2 Interface Library** for proper mode registration
- **Real-time obstacle detection** using data from the collision prevention node
- **Intelligent path planning** around obstacles
- **Safety features** including emergency stops and fail-safes

## Features

### Navigation
- Single waypoint and multi-waypoint path following
- Configurable waypoint arrival tolerance
- Automatic mission progression through waypoints
- Mission completion detection

### Collision Avoidance
- Real-time obstacle distance monitoring (72-sector coverage)
- Dynamic path replanning around obstacles
- Configurable safety distances and guidance angles
- Emergency stop when no safe path is available
- Graceful handling of sensor data timeouts

### Safety
- Integration with PX4's safety systems
- Fail-safe modes when obstacle data is unavailable
- Configurable speed limits and safety margins
- Comprehensive logging and debugging

## Usage

### Build and Install

```bash
cd ~/ws_sensor_combined
colcon build --packages-select collision_flightmode
source install/setup.bash
```

### Launch the Flight Mode

```bash
# Basic launch
ros2 launch collision_flightmode collision_autonomous_mode.launch.py

# With custom parameters
ros2 launch collision_flightmode collision_autonomous_mode.launch.py \
  max_speed:=2.0 \
  collision_distance_threshold:=3.0 \
  waypoint_tolerance:=0.5
```

### Send Waypoints

**Single Waypoint:**
```bash
ros2 topic pub /collision_flightmode/waypoint geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'map'}, point: {x: 10.0, y: 5.0, z: -3.0}}"
```

**Multiple Waypoints (Path):**
```bash
ros2 topic pub /collision_flightmode/path nav_msgs/msg/Path \
  "{header: {frame_id: 'map'}, poses: [
    {pose: {position: {x: 5.0, y: 0.0, z: -3.0}}},
    {pose: {position: {x: 10.0, y: 5.0, z: -3.0}}},
    {pose: {position: {x: 0.0, y: 10.0, z: -2.0}}}
  ]}"
```

## Configuration

Key parameters in `config/collision_autonomous_mode.yaml`:

- `waypoint_tolerance`: Distance to consider waypoint reached (default: 1.0m)
- `max_speed`: Maximum flight speed (default: 3.0 m/s)
- `collision_distance_threshold`: Minimum distance to obstacles (default: 2.0m)
- `guidance_angle_deg`: Maximum deviation angle for avoidance (default: 30°)

## Integration Requirements

### Required Nodes
1. **Collision Prevention Node**: Provides obstacle distance data
2. **PX4 uXRCE-DDS Client**: For PX4 communication
3. **Micro XRCE-DDS Agent**: ROS2-PX4 bridge

### PX4 Configuration
The mode appears as "Collision Aware Auto" in QGroundControl and can be:
- Selected manually from the flight mode menu
- Triggered via RC switch assignment
- Activated via MAVLink commands

### Topics

**Subscribed:**
- `/collision_flightmode/waypoint` (geometry_msgs/PointStamped)
- `/collision_flightmode/path` (nav_msgs/Path)  
- `/px4_0/fmu/out/obstacle_distance_fused` (px4_msgs/ObstacleDistance)

**Published:**
- PX4 setpoints via PX4 ROS2 Interface Library

## Algorithm

### Collision Avoidance Logic
1. **Obstacle Detection**: Monitor 72-sector obstacle distance data
2. **Path Analysis**: Check if direct path to waypoint is blocked
3. **Safe Direction Search**: Find alternative paths within guidance angle
4. **Setpoint Modification**: Redirect to safe path or stop if necessary
5. **Mission Continuation**: Resume direct path when obstacles clear

### Safety Features
- **Data Timeout Handling**: Graceful degradation when sensor data is lost
- **Emergency Stops**: Immediate stop when surrounded by obstacles  
- **Speed Limiting**: Reduced speed in cluttered environments
- **Mission Abort**: Safe hover when mission cannot continue

## Example Mission Sequence

1. Launch collision prevention and flight mode nodes
2. Arm vehicle and select "Collision Aware Auto" mode  
3. Send waypoint(s) via ROS2 topics
4. Vehicle autonomously navigates while avoiding obstacles
5. Mode reports mission completion when all waypoints reached

This mode provides the collision-aware autonomous navigation you need while integrating seamlessly with PX4's flight mode system!