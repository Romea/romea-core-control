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
#include "romea_core_control/FrontRearData.hpp"

namespace romea
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
  FollowMe(
    const double & sampling_period,
    const Parameters & parameters);

  void setDesiredLateralDeviation(const double & desired_lat_dev);

  const double & getDesiredLateralDeviation()const;

  double computeAngularSpeed(
    const double & lateral_deviation,
    const double & course_deviation,
    const double & maximal_angular_speed,
    double & omega_d,
    double & theta_error);

  FrontRearData computeSteeringAngles(
    const double & wheelbase,
    const double & lateral_deviation,
    const double & course_deviation,
    const double & rear_steering_angle,
    const double & maximal_front_steering_angle,
    const double & maximal_rear_steering_angle,
    double & omega_d,
    double & theta_consigne,
    const double & vitesse,
    const double & yaw_rate_leader,
    const double & desired_longitudinal_distance);

  FrontRearData computeSteeringAngles(
    const double & wheelbase,
    const double & lateral_deviation,
    const double & course_deviation,
    const double & courbure,
    const double & speed,
    const double & rear_steering_angle,
    const double & rear_sliding_angle,
    const double & front_sliding_angle,
    const double & minimal_theta,
    const double & maximal_theta,
    const double & maximal_front_steering_angle,
    const double & maximal_rear_steering_angle,
    double & omega_d,
    double & theta_consigne);


  FrontRearData computeSteeringAngles(
    const double & wheelbase,
    const double & lateral_deviation,
    const double & course_deviation,
    const double & curvature,
    const double & speed,
    const double & rear_steering_angle,
    const double & rear_sliding_angle,
    const double & front_sliding_angle,
    const double & minimal_theta,
    const double & maximal_theta,
    const double & maximal_front_steering_angle,
    const double & maximal_rear_steering_angle,
    const double & courbe0,
    const double & courbe1,
    const double & courbe2,
    const double & lambda,
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

}  // namespace romea

#endif  // ROMEA_CORE_CONTROL__COMMAND__FOLLOWME_HPP_
