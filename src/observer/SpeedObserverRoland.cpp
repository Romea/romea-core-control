// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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

// std
#include <cmath>

// romea
#include "romea_core_control/observer/SpeedObserverRoland.hpp"

namespace romea::core
{

const double SpeedObserverRoland::DEFAULT_KD = -2.2;

//-----------------------------------------------------------------------------
SpeedObserverRoland::SpeedObserverRoland() : SpeedObserverRoland(DEFAULT_KD)
{
}

//-----------------------------------------------------------------------------
SpeedObserverRoland::SpeedObserverRoland(double kd)
: kd_(kd), longitudinal_deviation_obs_(0), leader_linear_speed_obs_(0), is_initialized_(false)
{
}

//-----------------------------------------------------------------------------
double SpeedObserverRoland::update(
  double delta_time,
  double longitudinal_deviation,
  double course_deviation,
  double follower_linear_speed,
  double follower_angular_speed)
{
  if (is_initialized_) {
    double error = longitudinal_deviation_obs_ - longitudinal_deviation;

    leader_linear_speed_obs_ =
      kd_ * error + follower_linear_speed * std::cos(course_deviation) +
      0 * longitudinal_deviation * follower_angular_speed * std::sin(course_deviation);

    double longitudinal_deviation_obs_dot =
      leader_linear_speed_obs_ - follower_linear_speed * std::cos(course_deviation) -
      0 * longitudinal_deviation * follower_angular_speed * std::sin(course_deviation);

    longitudinal_deviation_obs_ += delta_time * longitudinal_deviation_obs_dot;
  } else {
    longitudinal_deviation_obs_ = longitudinal_deviation;
    is_initialized_ = true;
  }

  if (leader_linear_speed_obs_ > 0) {
    return leader_linear_speed_obs_;
  }
  return 0.0;
}

//-----------------------------------------------------------------------------
double SpeedObserverRoland::update(
  double delta_time,
  double longitudinal_deviation,
  double course_deviation,
  double follower_linear_speed)
{
  return update(delta_time, longitudinal_deviation, course_deviation, follower_linear_speed, 0.);
}

//-----------------------------------------------------------------------------
void SpeedObserverRoland::reset()
{
  leader_linear_speed_obs_ = 0;
  is_initialized_ = false;
}

}  // namespace romea::core
