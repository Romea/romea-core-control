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

// Eigen
#include <Eigen/Dense>

// std
#include <cmath>

// romea core
#include <romea_core_common/math/Algorithm.hpp>

// local
#include "romea_core_control/command/FollowTrajectoryFrontRearDecoupled.hpp"

namespace romea::core
{

FollowTrajectoryFrontRearDecoupled::FollowTrajectoryFrontRearDecoupled(
  double wheel_base, Parameters parameters)
: wheelbase_(wheel_base), params_(parameters)
{
}

FrontRearData FollowTrajectoryFrontRearDecoupled::computeSteeringAngles(
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
  double /*desired_course_deviation*/)
{
  double front_steering_angle_command = computeFrontSteeringAngle_(
    -lateral_deviation - desired_lateral_deviation,
    course_deviation,
    curvature,
    future_curvature,
    linear_speed,
    front_steering_angle,
    -rear_steering_angle,
    -front_sliding_angle,
    rear_sliding_angle);

  double rear_steering_angle_command = computeRearSteeringAngle_(
    lateral_deviation - desired_lateral_deviation,
    course_deviation,
    linear_speed,
    rear_sliding_angle);

  double clamped_front_steering_angle_command = clamp(
    front_steering_angle_command, -front_maximal_steering_angle, front_maximal_steering_angle);
  double clamped_rear_steering_angle_command =
    clamp(rear_steering_angle_command, -rear_maximal_steering_angle, rear_maximal_steering_angle);

  if (
    (std::abs(front_steering_angle_command) > front_maximal_steering_angle) &&
    (std::abs(rear_steering_angle_command) > front_maximal_steering_angle) &&
    ((front_steering_angle_command * front_steering_angle_command) > 0)) {
    rear_steering_angle_command = std::atan(
      std::tan(clamped_rear_steering_angle_command) +
      std::tan(clamped_front_steering_angle_command) - std::tan(front_steering_angle_command));
  }

  clamped_rear_steering_angle_command =
    clamp(rear_steering_angle_command, -rear_maximal_steering_angle, rear_maximal_steering_angle);

  return {clamped_front_steering_angle_command, clamped_rear_steering_angle_command};
}

double FollowTrajectoryFrontRearDecoupled::computeFrontSteeringAngle_(
  double lateral_deviation,  // EcLat    y
  double course_deviation,   // EcAng    theta
  double curvature,          // CR obsdynglob
  double future_curvature,   // courbure future
  double linear_speed,       // vitesse  v
  double /*front_steering_angle*/,
  double rear_steering_angle,
  double front_sliding_angle,  // BetaF obsdynglob
  double rear_sliding_angle)   // BetaR obsdynglob
{
  std::cout << " front dynpred " << std::endl;
  std::cout << "l " << lateral_deviation << " co " << course_deviation << " cu " << curvature
            << " cuf " << future_curvature << std::endl;
  std::cout << "s " << linear_speed << " rs " << rear_steering_angle << " fsb "
            << front_sliding_angle << " rsb  " << rear_sliding_angle << std::endl;

  double kf = params_.front_kp;

  double thetaT2 = course_deviation + rear_steering_angle + rear_sliding_angle;

  double B1 = (std::sin(thetaT2) /
               (std::cos(rear_steering_angle + rear_sliding_angle) * std::cos(course_deviation))) -
              std::tan(rear_steering_angle + rear_sliding_angle);

  double mode_courbure = wheelbase_ * curvature * std::cos(course_deviation);

  double front_lateral_deviation;
  if (std::abs(mode_courbure) < .9) {
    double alpha = std::asin(mode_courbure);
    double e = 0;
    if (std::abs(curvature) > 0.001) {
      e = (1 - std::cos(alpha)) / curvature;
    }
    front_lateral_deviation = lateral_deviation - wheelbase_ * sin(course_deviation) + e;
  } else {
    double alpha = std::atan(
      (1 / curvature - lateral_deviation - wheelbase_ * std::sin(course_deviation)) /
      (wheelbase_ * std::cos(course_deviation)));
    front_lateral_deviation =
      wheelbase_ * std::cos(course_deviation) / std::cos(alpha) - (1 / curvature);
  }

  // double YF2_=front_lateral_deviation;
  double lambda2 = (future_curvature * cos(thetaT2)) / (1 - future_curvature * lateral_deviation);

  double front_command_dyn_ = 0;
  if (linear_speed > 0.5) {
    front_command_dyn_ =
      std::atan(
        ((wheelbase_ * lambda2) / (std::cos(rear_steering_angle + rear_sliding_angle))) +
        ((kf * front_lateral_deviation) /
         (linear_speed * std::cos(rear_steering_angle + rear_sliding_angle) *
          (std::cos(course_deviation)))) -
        B1) +
      front_sliding_angle;
  }

  std::cout << "front_command_dyn_  " << front_command_dyn_ << std::endl;
  return front_command_dyn_;
}

double FollowTrajectoryFrontRearDecoupled::computeRearSteeringAngle_(
  double lateral_deviation, double course_deviation, double linear_speed, double rear_sliding_angle)
{
  std::cout << " rear dynpred " << std::endl;
  std::cout << "l " << lateral_deviation << " co " << course_deviation << " s " << linear_speed
            << " rsb " << rear_sliding_angle << std::endl;

  double rear_command_dyn_ = 0;
  if (std::abs(linear_speed) > 0.8) {
    double ConvInter = (-params_.rear_kp * lateral_deviation) / (linear_speed);
    if (ConvInter > 0.9) {
      ConvInter = 0.9;
    }
    if (ConvInter < -0.9) {
      ConvInter = -0.9;
    }

    rear_command_dyn_ = std::asin(ConvInter) - course_deviation - rear_sliding_angle;
  }

  return rear_command_dyn_;
}

}  // namespace romea::core
