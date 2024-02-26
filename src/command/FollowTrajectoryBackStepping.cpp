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
#include <iostream>

// romea core
#include "romea_core_common/math/Algorithm.hpp"

// local
#include "romea_core_control/command/FollowTrajectoryBackStepping.hpp"


// namespace
// {
// const double SKID_STEERING_MAXIMAL_OMEGA_D = std::tan(50 * M_PI / 180);
// const double SKID_STEERING_MAXIMAL_INTEDRATED_OMEGA_D = 0.75;
// }

namespace romea
{
namespace core
{


//-----------------------------------------------------------------------------
FollowTrajectoryBackStepping::FollowTrajectoryBackStepping(
  const double & sampling_period,
  const double & wheelbase,
  const Parameters & parameters)
: wheelbase_(wheelbase),
  sampling_period_(sampling_period),
  kp_(parameters.kp),
  ki_(parameters.ki),
  kd_(parameters.kd),
  kdd_(parameters.kdd),
  omega_d_integral_(0),
  previous_desired_lateral_deviation_(0)
{
}

//-----------------------------------------------------------------------------
void FollowTrajectoryBackStepping::reset()
{
  omega_d_integral_ = 0;
}

//-----------------------------------------------------------------------------
double FollowTrajectoryBackStepping::computeAngularSpeed(
  const double & lateral_deviation,
  const double & course_deviation,
  const double & curvature,
  const double & linear_speed,
  const double & maximal_angular_speed,
  const double & desired_lateral_deviation,
  double & omega_d)
{

  if (std::abs(desired_lateral_deviation - previous_desired_lateral_deviation_) >
    std::numeric_limits<double>::epsilon())
  {
    reset();
  }
  previous_desired_lateral_deviation_ = desired_lateral_deviation;


  omega_d = std::atan2(kp_ * (lateral_deviation - desired_lateral_deviation), 1);

  if (std::abs(omega_d) > maximal_omega_d_) {
    omega_d = copysign(maximal_omega_d_, omega_d);
  }

  omega_d_integral_ += sampling_period_ * (course_deviation - omega_d);

  if (std::abs(omega_d_integral_) > maximal_omega_d_integral_) {
    omega_d_integral_ = copysign(maximal_omega_d_integral_, omega_d_integral_);
  }

  double angular_speed_command = kd_ * (course_deviation - omega_d) + ki_ * omega_d_integral_ -
    (linear_speed * curvature * std::cos(course_deviation)) / (1 - curvature * lateral_deviation);

  if (std::abs(angular_speed_command) > maximal_angular_speed) {
    angular_speed_command = std::copysign(maximal_angular_speed, angular_speed_command);
  }

  return angular_speed_command;
}


//-----------------------------------------------------------------------------
FrontRearData FollowTrajectoryBackStepping::computeSteeringAngles(
  const double & lateral_deviation,
  const double & course_deviation,
  const double & curvature,
  const double & rear_steering_angle,
  const double & rear_sliding_angle,
  const double & front_sliding_angle,
  const double & minimal_theta,
  const double & maximal_theta,
  const double & maximal_front_steering_angle,
  const double & maximal_rear_steering_angle,
  const double & desired_lateral_deviation,
  double & omega_d,
  double & theta_consigne)
{
  // double alpha = 1 - curvature * (lateral_deviation + 1.35);
  // omega_d = atan(kp_ * (lateral_deviation - desired_lat_dev_) / alpha);
  omega_d = kp_ * (lateral_deviation - desired_lateral_deviation);


  if (std::abs(omega_d) > maximal_omega_d_) {
    omega_d = copysign(maximal_omega_d_, omega_d);
  }

  if (omega_d < minimal_theta) {
    omega_d = minimal_theta;
  }

  if (omega_d > maximal_theta) {
    omega_d = maximal_theta;
  }

  double Thet2 = course_deviation + rear_steering_angle + rear_sliding_angle;
  double EpsThet = -(Thet2 - omega_d);

  // braq_F = std::atan(
  // kd_ * EpsThet * wheelbase * cos(Thet2) / cos(rear_steering_angle) + tan(rear_steering_angle));
  double front_steering_angle_command = std::atan(
    (kd_ * EpsThet + curvature) * wheelbase_ * std::cos(Thet2) /
    cos(rear_steering_angle + rear_sliding_angle) +
    std::tan(rear_steering_angle + rear_sliding_angle)) - front_sliding_angle;

  theta_consigne = 0;
  if ((fabs(omega_d) > 5 * M_PI / 180) || (fabs(EpsThet) > 7.5 * M_PI / 180)) {
    theta_consigne = omega_d;
  }

  double ThetaError2 = theta_consigne - course_deviation;
  //  integrated_omega_ += 0.1*(tan(course_deviation)-omega_d);
  //  if (fabs(integrated_omega_) > 0.75)
  //    integrated_omega_ = copysign(0.75, integrated_omega_);

  //  if(fabs(speed) < 0.1)
  //    integrated_omega_ = 0;

  //  braq_R=-course_deviation-(1/kd_)*( kdd_*ThetaError2 - kd_*omega_d);
  double rear_steering_angle_command = -course_deviation - rear_sliding_angle -
    (1 / kd_) * (kdd_ * ThetaError2 - kd_ * omega_d);

  // std::cout << " new command " << omega_d << " " << Thet2 << " " << EpsThet << " " << braq_F <<
  //   " " << theta_consigne << " " << ThetaError2 << " " << braq_R << std::endl;

  if (std::abs(rear_steering_angle_command) > maximal_rear_steering_angle) {
    rear_steering_angle_command = copysign(
      maximal_rear_steering_angle,
      rear_steering_angle_command);
  }

  if (std::abs(front_steering_angle_command) > maximal_front_steering_angle) {
    front_steering_angle_command = copysign(
      maximal_front_steering_angle,
      front_steering_angle_command);
  }

  return {front_steering_angle_command, rear_steering_angle_command};
}


}  // namespace core
}  // namespace romea
