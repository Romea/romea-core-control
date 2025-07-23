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

#ifndef ROMEA_CORE_CONTROL__COMMAND__KEEPINTERDISTANCE_HPP_
#define ROMEA_CORE_CONTROL__COMMAND__KEEPINTERDISTANCE_HPP_

// std
#include <vector>

// romea
#include "romea_core_common/math/Algorithm.hpp"
#include "romea_core_common/signal/FirstOrderButterworth.hpp"

namespace romea::core
{

class KeepInterdistance
{
public:
  explicit KeepInterdistance(double sampling_period);

  double computeFollowerSpeed(
    double desired_interdistance,
    double interdistance,
    double leader_linear_speed,
    double follower_maximal_linear_speed);

  double computeFollowerSpeed(
    double desired_interdistance,
    double interdistance,
    double leader_linear_speed,
    double follower_maximal_linear_speed,
    double follower_lat_dev,
    double follower_ang_dev,
    double courbure,
    double follower_linear_speed);

  double computeFollowerSpeed(
    double desired_interdistance,
    double interdistance,
    double follower_lateral_deviation,
    double follower_linear_speed,
    double follower_maximal_linear_speed);

  double computeFollowerSpeed(
    double desired_interdistance,
    double desired_lateral_deviation,
    double follower_interdistance,
    double leader_interdistance,
    double leader_linear_speed,
    double follower_lateral_deviation,
    double follower_course_deviation,
    double follower_linear_speed,
    double follower_maximal_linear_speed,
    double desired_angular_deviation,
    double yaw_rate_leader);

  double computeFollowerSpeed(
    double desired_interdistance,
    double interdistance,
    double leader_lateral_deviation,
    double leader_course_deviation,
    double leader_curvature,
    double leader_linear_speed,
    double leader_rear_streering_angle,
    double leader_rear_sliding_angle,
    double follower_lateral_deviation,
    double follower_course_deviation,
    double follower_curvature,
    double follower_linear_speed,
    double follower_rear_streering_angle,
    double follower_rear_sliding_angle,
    double follower_maximal_linear_speed);

  void reset();

private:
  //  double clampLinearSpeed_(double linear_speed,
  //                           double maximal_linear_speed);

  double offsetFreeLinearSpeed_(double linear_speed);

  void updateLongitudinalDeviation_(
    double interdistance, double desired_interdistance, double follower_linear_speed);

private:
  double sampling_period_;

  double KI_;
  double interdistance_error_;
  double integrated_longitudinal_deviation_;
};

}  // namespace romea::core

#endif  // ROMEA_CORE_CONTROL__COMMAND__KEEPINTERDISTANCE_HPP_
