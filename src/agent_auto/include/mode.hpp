/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

#include <rclcpp/rclcpp.hpp>
#include <multi_agent_planner_msgs/msg/trajectory.hpp>
#include <px4_ros_com/frame_transforms.h>

#include <Eigen/Core>

using namespace std::chrono_literals; // NOLINT

static const std::string kName = "Agent Auto";


class AgentAutoMode : public px4_ros2::ModeBase
{
public:
  explicit AgentAutoMode(rclcpp::Node & node)
  : ModeBase(node, Settings{kName, false})
  {
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    
    _trajectory_subscriber = node.create_subscription<multi_agent_planner_msgs::msg::Trajectory>(
      "/agent_0/traj_full", 10,
      [this](const multi_agent_planner_msgs::msg::Trajectory::SharedPtr msg) {
        this->trajectoryCallback(msg);
      });
  }

  ~AgentAutoMode() override = default;

  void onActivate() override
  {
    _activation_time = node().get_clock()->now();
    _has_trajectory = false;
    RCLCPP_INFO(node().get_logger(), "Agent Auto mode activated - listening for trajectory");
  }

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    if (!_has_trajectory) {
      // Hover in place until trajectory is received
      const Eigen::Vector3f velocity{0.0f, 0.0f, 0.0f};
      _trajectory_setpoint->update(velocity);
      return;
    }

    // Use the latest received trajectory control
    _trajectory_setpoint->update(_current_velocity, _current_acceleration);
  }

  void trajectoryCallback(const multi_agent_planner_msgs::msg::Trajectory::SharedPtr msg)
  {
    if (msg->states.size() > 1) {
      // Use the second state (index 1) for immediate control commands
      const auto& state = msg->states[1];
      
      // Extract velocity (assuming it's in ENU frame from trajectory planner)
      if (state.velocity.size() >= 3) {
        Eigen::Vector3d vel_enu(
          state.velocity[0],
          state.velocity[1], 
          state.velocity[2]
        );
        
        // Convert ENU to NED for PX4
        Eigen::Vector3d vel_ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(vel_enu);
        _current_velocity = Eigen::Vector3f(
          static_cast<float>(vel_ned.x()),
          static_cast<float>(vel_ned.y()),
          static_cast<float>(vel_ned.z())
        );
      }
      
      // Extract acceleration (assuming it's in ENU frame from trajectory planner)
      if (state.acceleration.size() >= 3) {
        Eigen::Vector3d acc_enu(
          state.acceleration[0],
          state.acceleration[1],
          state.acceleration[2]
        );
        
        // Convert ENU to NED for PX4
        Eigen::Vector3d acc_ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(acc_enu);
        _current_acceleration = Eigen::Vector3f(
          static_cast<float>(acc_ned.x()),
          static_cast<float>(acc_ned.y()),
          static_cast<float>(acc_ned.z())
        );
      }
      
      _current_yaw = static_cast<float>(msg->yaw);
      _has_trajectory = true;
      
      RCLCPP_DEBUG(node().get_logger(), 
                  "Trajectory (NED): vel=[%.3f, %.3f, %.3f] acc=[%.3f, %.3f, %.3f] yaw=%.3f", 
                  _current_velocity.x(), _current_velocity.y(), _current_velocity.z(),
                  _current_acceleration.x(), _current_acceleration.y(), _current_acceleration.z(),
                  _current_yaw);
    } else {
      RCLCPP_WARN(node().get_logger(), "Received trajectory with insufficient states");
    }
  }

private:
  rclcpp::Time _activation_time{};
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
  rclcpp::Subscription<multi_agent_planner_msgs::msg::Trajectory>::SharedPtr _trajectory_subscriber;
  
  bool _has_trajectory{false};
  Eigen::Vector3f _current_velocity{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f _current_acceleration{0.0f, 0.0f, 0.0f};
  float _current_yaw{0.0f};
};

class AgentAutoExecutor : public px4_ros2::ModeExecutorBase
{
public:
  AgentAutoExecutor(rclcpp::Node & node, px4_ros2::ModeBase & owned_mode)
  : ModeExecutorBase(node, px4_ros2::ModeExecutorBase::Settings{}, owned_mode),
    _node(node)
  {
    RCLCPP_INFO(_node.get_logger(), "Agent Auto Executor initialized");
  }

  enum class State
  {
    Reset,
    AgentAuto,
    RTL,
    WaitUntilDisarmed,
  };

  void onActivate() override
  {
    runState(State::AgentAuto, px4_ros2::Result::Success);
  }

  void onDeactivate(DeactivateReason reason) override
  {
  }

  void runState(State state, px4_ros2::Result previous_result)
  {
    if (previous_result != px4_ros2::Result::Success) {
      RCLCPP_ERROR(
        _node.get_logger(), "State %i: previous state failed: %s", (int)state,
        resultToString(previous_result));
      return;
    }

    RCLCPP_DEBUG(_node.get_logger(), "Executing state %i", (int)state);

    switch (state) {
      case State::Reset:
        break;

      case State::AgentAuto:
        scheduleMode(
          ownedMode().id(), [this](px4_ros2::Result result) {
            runState(State::RTL, result);
          });
        break;

      case State::RTL:
        rtl([this](px4_ros2::Result result) {runState(State::WaitUntilDisarmed, result);});
        break;

      case State::WaitUntilDisarmed:
        waitUntilDisarmed(
          [this](px4_ros2::Result result) {
            RCLCPP_INFO(_node.get_logger(), "All states complete (%s)", resultToString(result));
          });
        break;
    }
  }

private:
  rclcpp::Node & _node;
};
