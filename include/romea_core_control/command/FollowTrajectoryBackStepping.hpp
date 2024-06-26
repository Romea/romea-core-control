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


#ifndef ROMEA_CORE_CONTROL__COMMAND__FOLLOWTRAJECTORYBACKSTEPPING_HPP_
#define ROMEA_CORE_CONTROL__COMMAND__FOLLOWTRAJECTORYBACKSTEPPING_HPP_

// std
#include <utility>

// romea
#include "romea_core_control/FrontRearData.hpp"

namespace romea
{
namespace core
{

class FollowTrajectoryBackStepping
{
public:
  struct Parameters
  {
    double kp;
    double kd;
    double ki;
    double iclamp;
    double kdd;
    double maximal_omega_d;
  };

public:
  FollowTrajectoryBackStepping(
    const double & sampling_period,
    const double & wheelbase,
    const Parameters & parameters);

  double computeAngularSpeed(
    const double & lateral_deviation,
    const double & course_deviation,
    const double & curvature,
    const double & linear_speed,
    const double & maximal_angular_speed,
    const double & desired_lateral_deviation,
    double & omega_d);

  FrontRearData computeSteeringAngles(
    const double & lateral_deviation,
    const double & course_deviation,
    const double & curvature,
    const double & rear_steering_angle,
    const double & rear_sliding_angle,
    const double & front_sliding_angle,
    const double & maximal_front_steering_angle,
    const double & maximal_rear_steering_angle,
    const double & desired_lateral_deviation,
    double & omega_d,
    double & theta_consigne);

  void reset();

private:
  void update_integral_(
    const double & desired_lateral_deviation,
    const double & error);

private:
  double wheelbase_;
  double sampling_period_;

  double kp_;
  double ki_;
  double kd_;
  double i_;
  double iclamp_;
  double kdd_;
  double maximal_omega_d_;

  // double desired_lat_dev_;
  double omega_d_error_integral_;
  double last_desired_lateral_deviation_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_CONTROL__COMMAND__FOLLOWME_HPP_
