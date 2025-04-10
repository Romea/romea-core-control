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

#ifndef ROMEA_CORE_CONTROL__COMMAND__FOLLOWME_HPP_
#define ROMEA_CORE_CONTROL__COMMAND__FOLLOWME_HPP_

// std
#include <utility>

// romea
#include "romea_core_control/CommandsData.hpp"

namespace romea::core
{

class FollowMe
{
public:
  struct Parameters
  {
    double kp;
    double kd;
    double ki;
    double kdd;
  };

public:
  FollowMe(double sampling_period, const Parameters & parameters);

  void setDesiredLateralDeviation(double desired_lat_dev);

  double getDesiredLateralDeviation() const;

  double computeAngularSpeed(
    double lateral_deviation,
    double course_deviation,
    double maximal_angular_speed,
    double & omega_d,
    double & theta_error);

  FrontRearData computeSteeringAngles(
    double wheelbase,
    double lateral_deviation,
    double course_deviation,
    double rear_steering_angle,
    double maximal_front_steering_angle,
    double maximal_rear_steering_angle,
    double & omega_d,
    double & theta_consigne,
    double vitesse,
    double yaw_rate_leader,
    double desired_longitudinal_distance);

  FrontRearData computeSteeringAngles(
    double wheelbase,
    double lateral_deviation,
    double course_deviation,
    double courbure,
    double speed,
    double rear_steering_angle,
    double rear_sliding_angle,
    double front_sliding_angle,
    double minimal_theta,
    double maximal_theta,
    double maximal_front_steering_angle,
    double maximal_rear_steering_angle,
    double & omega_d,
    double & theta_consigne);

  FrontRearData computeSteeringAngles(
    double wheelbase,
    double lateral_deviation,
    double course_deviation,
    double curvature,
    double speed,
    double rear_steering_angle,
    double rear_sliding_angle,
    double front_sliding_angle,
    double minimal_theta,
    double maximal_theta,
    double maximal_front_steering_angle,
    double maximal_rear_steering_angle,
    double courbe0,
    double courbe1,
    double courbe2,
    double lambda,
    double & omega_d,
    double & theta_consigne);

  void reset();

private:
  double sampling_period_;

  double kp_;
  double ki_;
  double kd_;
  double kdd_;

  double desired_lat_dev_;
  double integrated_omega_;
};

}  // namespace romea::core

#endif  // ROMEA_CORE_CONTROL__COMMAND__FOLLOWME_HPP_
