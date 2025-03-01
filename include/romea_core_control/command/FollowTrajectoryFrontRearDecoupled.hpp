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

#ifndef ROMEA_CORE_CONTROL__COMMAND__FOLLOWTRAJECTORYFRONTREARDECOUPLED_HPP_
#define ROMEA_CORE_CONTROL__COMMAND__FOLLOWTRAJECTORYFRONTREARDECOUPLED_HPP_

// romea
#include "romea_core_control/FrontRearData.hpp"

namespace romea::core
{

class FollowTrajectoryFrontRearDecoupled
{
public:
  struct Parameters
  {
    double front_kp;
    double rear_kp;
  };

public:
  FollowTrajectoryFrontRearDecoupled(double wheel_base, Parameters parameters);

FrontRearData computeSteeringAngles(
  double lateral_deviation,
  double course_deviation,
  double curvature,
  double future_curvature,
  double linear_speed,
  double front_steering_angle,
  double rear_steering_angle,
  double front_sliding_angle,
  double rear_sliding_angle,
  double front_maximal_steering_angle,
  double rear_maximal_steering_angle,
  double desired_lateral_deviation,
  double desired_course_deviation) const;

private:
  double computeFrontSteeringAngle_(
    double lateral_deviation,
    double course_deviation,
    double curvature,
    double future_curvature,
    double linear_speed,
    double front_steering_angle,
    double rear_steering_angle,
    double front_sliding_angle,
    double rear_sliding_angle) const;

  double computeRearSteeringAngle_(
    double lateral_deviation,
    double course_deviation,
    double linear_speed,
    double rear_sliding_angle) const;

private:
  double wheelbase_;
  Parameters params_;
};

}  // namespace romea::core

#endif  // ROMEA_CORE_CONTROL__COMMAND__FOLLOWTRAJECTORYCLASSICSLIDING_HPP_
