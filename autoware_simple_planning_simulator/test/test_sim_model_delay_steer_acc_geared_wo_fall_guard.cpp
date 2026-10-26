// Copyright 2026 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc_geared_wo_fall_guard.hpp"

#include <autoware_vehicle_msgs/msg/gear_command.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace autoware::simulator::simple_planning_simulator
{
namespace
{
constexpr double vx_lim = 50.0;
constexpr double steer_lim = 0.6;
constexpr double vx_rate_lim = 7.0;
constexpr double steer_rate_lim = 5.0;
constexpr double wheelbase = 2.79;
constexpr double dt = 0.025;
constexpr double acc_delay = 0.1;
constexpr double acc_time_constant = 0.1;
constexpr double steer_delay = 0.1;
constexpr double steer_time_constant = 0.1;
constexpr double steer_dead_band = 0.0;
constexpr double steer_bias = 0.0;
constexpr double debug_acc_scaling_factor = 1.0;
constexpr double debug_steer_scaling_factor = 1.0;

SimModelDelaySteerAccGearedWoFallGuard make_model(const double k_us)
{
  return SimModelDelaySteerAccGearedWoFallGuard(
    vx_lim, steer_lim, vx_rate_lim, steer_rate_lim, wheelbase, dt, acc_delay, acc_time_constant,
    steer_delay, steer_time_constant, steer_dead_band, steer_bias, debug_acc_scaling_factor,
    debug_steer_scaling_factor, k_us);
}

// Hold a constant speed and steering angle, and return the yaw rate.
double steady_yaw_rate(SimModelInterface & model, const double vx, const double steer)
{
  Eigen::VectorXd state(7);
  state << 0.0, 0.0, 0.0, vx, steer, 0.0, 0.0;
  model.setState(state);
  Eigen::VectorXd input(4);
  input << 0.0, autoware_vehicle_msgs::msg::GearCommand::DRIVE, 0.0, steer;
  model.setInput(input);
  return model.getWz();
}
}  // namespace

TEST(SimModelDelaySteerAccGearedWoFallGuard, UndersteerGradientReducesYawRate)
{
  constexpr double vx = 10.0;
  constexpr double steer = 0.1;
  constexpr double k_us = 0.01;

  auto kinematic = make_model(0.0);
  EXPECT_DOUBLE_EQ(steady_yaw_rate(kinematic, vx, steer), vx * std::tan(steer) / wheelbase);

  auto understeer = make_model(k_us);
  EXPECT_DOUBLE_EQ(
    steady_yaw_rate(understeer, vx, steer), vx * std::tan(steer) / (wheelbase + k_us * vx * vx));
}

}  // namespace autoware::simulator::simple_planning_simulator
