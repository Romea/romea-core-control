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

#ifndef ROMEA_CORE_CONTROL__COMMAND__FOLLOW_TRAJECTORY_SKID_SLIDING_HPP_
#define ROMEA_CORE_CONTROL__COMMAND__FOLLOW_TRAJECTORY_SKID_SLIDING_HPP_

// romea
#include "romea_core_control/FrontRearData.hpp"

namespace romea::core
{

class FollowTrajectorySkidSliding
{
public:
  struct Parameters
  {
    double lateral_kp;
    double course_kp;
  };

public:
  FollowTrajectorySkidSliding(double wheelbase, const Parameters & parameters);

  double computeAngularSpeed(
    double lateral_deviation,
    double course_deviation,
    double curvature,
    double linear_speed,
    double linear_speed_disturbance,
    double angular_speed_disturbance,
    double sliding_angle,
    double maximal_angular_speed,
    double desired_lateral_deviation,
    double & target_course) const;

private:
  double wheelbase_;
  Parameters params_;
};

}  // namespace romea::core

#endif
