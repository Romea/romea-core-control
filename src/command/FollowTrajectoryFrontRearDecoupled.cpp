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
  double /*desired_course_deviation*/) const
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

  // this handles a singularity of the control command when the front steering is saturated
  bool is_singularity =
    std::abs(front_steering_angle_command) > front_maximal_steering_angle &&
    std::abs(rear_steering_angle_command) > front_maximal_steering_angle &&
    std::signbit(front_steering_angle_command) == std::signbit(rear_steering_angle_command);
  if (is_singularity) {
    rear_steering_angle_command = std::atan(
      std::tan(clamped_rear_steering_angle_command) +
      std::tan(clamped_front_steering_angle_command) - std::tan(front_steering_angle_command));
  }

  clamped_rear_steering_angle_command =
    clamp(rear_steering_angle_command, -rear_maximal_steering_angle, rear_maximal_steering_angle);

  return {clamped_front_steering_angle_command, clamped_rear_steering_angle_command};
}

double FollowTrajectoryFrontRearDecoupled::computeFrontSteeringAngle_(
  double lateral_deviation,
  double course_deviation,
  double curvature,
  double future_curvature,
  double linear_speed,
  double /*front_steering_angle*/,
  double rear_steering_angle,
  double front_sliding_angle,
  double rear_sliding_angle) const
{
  // std::cout << " front dynpred " << std::endl;
  // std::cout << "l " << lateral_deviation << " co " << course_deviation << " cu " << curvature
  //           << " cuf " << future_curvature << std::endl;
  // std::cout << "s " << linear_speed << " rs " << rear_steering_angle << " fsb "
  //           << front_sliding_angle << " rsb  " << rear_sliding_angle << std::endl;

  double thetaT2 = course_deviation + rear_steering_angle + rear_sliding_angle;
  double cos_course_deviation = std::cos(course_deviation);
  double mode_courbure = wheelbase_ * curvature * cos_course_deviation;

  double front_lateral_deviation = 0.;
  if (std::abs(mode_courbure) < .9) {
    double alpha = std::asin(mode_courbure);
    double e = 0;
    if (std::abs(curvature) > 1e-3) {
      e = (1 - std::cos(alpha)) / curvature;
    }
    front_lateral_deviation = lateral_deviation - wheelbase_ * sin(course_deviation) + e;
  } else {
    double alpha = std::atan(
      (1 / curvature - lateral_deviation - wheelbase_ * std::sin(course_deviation)) /
      (wheelbase_ * cos_course_deviation));
    front_lateral_deviation = wheelbase_ * cos_course_deviation / std::cos(alpha) - (1 / curvature);
  }

  double lambda2 = future_curvature * cos(thetaT2) / (1 - future_curvature * lateral_deviation);

  double front_command_dyn_ = 0;
  if (std::abs(linear_speed) > 1e-3) {
    double cos_rear_angle = std::cos(rear_steering_angle + rear_sliding_angle);
    double B1 = std::sin(thetaT2) / (cos_rear_angle * cos_course_deviation) -
                std::tan(rear_steering_angle + rear_sliding_angle);
    front_command_dyn_ = std::atan(
                           (wheelbase_ * lambda2 / cos_rear_angle) +
                           (params_.front_kp * front_lateral_deviation /
                            (linear_speed * cos_rear_angle * cos_course_deviation)) -
                           B1) +
                         front_sliding_angle;
  }

  // std::cout << "front_command_dyn_  " << front_command_dyn_ << std::endl;
  return front_command_dyn_;
}

double FollowTrajectoryFrontRearDecoupled::computeRearSteeringAngle_(
  double lateral_deviation,
  double course_deviation,
  double linear_speed,
  double rear_sliding_angle) const
{
  double rear_command_dyn_ = 0;
  if (std::abs(linear_speed) > 1e-3) {
    double conv_inter = -params_.rear_kp * lateral_deviation / linear_speed;
    // saturate conv_inter to stay in the domain of asin()
    conv_inter = clamp(conv_inter, -1., 1.);
    rear_command_dyn_ = std::asin(conv_inter) - course_deviation - rear_sliding_angle;
  }

  return rear_command_dyn_;
}

}  // namespace romea::core
