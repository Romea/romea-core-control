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
#include <iostream>

// romea core
#include "romea_core_common/math/Algorithm.hpp"

// local
#include "romea_core_control/command/FollowTrajectoryClassicSliding.hpp"

// namespace {
// const double DEFAULT_KD = 0.7;
// const double DEFAULT_KP =DEFAULT_KD*DEFAULT_KD/4;
// const double DEFAULT_KD2 = 0.5;
// }

namespace romea::core
{

//-----------------------------------------------------------------------------
FollowTrajectoryClassicSliding::FollowTrajectoryClassicSliding(
  double whee_base, const Parameters & parameters)
: wheelbase_(whee_base),
  KD_(parameters.front_kp),
  KP_(KD_ * KD_ / 4),
  KD2_(parameters.rear_kp),
  DeltaM_ar(0)
{
}

//-----------------------------------------------------------------------------
void FollowTrajectoryClassicSliding::setFrontKP(double kp)
{
  KD_ = kp;
  KP_ = (KD_ * KD_ / 4.);
}
//-----------------------------------------------------------------------------
FrontRearData FollowTrajectoryClassicSliding::computeSteeringAngles(
  double lateral_deviation,
  double course_deviation,
  double curvature,
  double front_sliding_angle,
  double rear_sliding_angle,
  double rear_steering_angle,
  double front_maximal_steering_angle,
  double rear_maximal_steering_angle,
  double desired_lateral_deviation,
  double desired_course_deviation)
{
  // compute front steering angle
  double front_steering_angle_ = computeFrontSteeringAngle_(
    lateral_deviation,
    course_deviation,
    curvature,
    -front_sliding_angle,
    rear_sliding_angle + rear_steering_angle,
    desired_lateral_deviation);

  front_steering_angle_ =
    clamp(front_steering_angle_, -front_maximal_steering_angle, front_maximal_steering_angle);

  // compute rear steering angle
  double rear_steering_angle_ = computeRearSteeringAngle_(
    lateral_deviation,
    course_deviation,
    curvature,
    rear_sliding_angle,
    desired_lateral_deviation,
    desired_course_deviation);

  rear_steering_angle_ =
    clamp(rear_steering_angle_, -rear_maximal_steering_angle, rear_maximal_steering_angle);

  return {front_steering_angle_, rear_steering_angle_};
}

//-----------------------------------------------------------------------------
double FollowTrajectoryClassicSliding::computeFrontSteeringAngle_(
  double lateral_deviation,
  double course_deviation,
  double curvature,
  double front_sliding_angle,
  double rear_sliding_angle,
  double desired_lateral_deviation)
{
  double a1 = course_deviation + rear_sliding_angle + DeltaM_ar;
  double a2 = 1 - curvature * lateral_deviation;
  double a3 = -KD_ * a2 * std::tan(a1) - KP_ * (lateral_deviation - desired_lateral_deviation) +
              curvature * a2 * ((std::tan(a1)) * (std::tan(a1)));

  return std::atan(
           (wheelbase_ / std::cos(rear_sliding_angle + DeltaM_ar)) *
             (curvature * std::cos(a1) / a2 +
              a3 * ((std::cos(a1)) * (std::cos(a1)) * (std::cos(a1))) / (a2 * a2)) +
           std::tan(rear_sliding_angle)) +
         front_sliding_angle;
}

//------------------------------------------------------------------------------
double FollowTrajectoryClassicSliding::computeRearSteeringAngle_(
  double lateral_deviation,
  double course_deviation,
  double curvature,
  double rear_sliding_angle,
  double desired_lateral_deviation,
  double desired_course_deviation)
{
  // if rear Kd is not defined, use a null angle
  if (std::isnan(KD2_)) {
    return 0;
  }

  double rear_steering_angle_ = -course_deviation - rear_sliding_angle;

  // if it is a straight line
  if (std::abs(curvature) <= 0.001) {
    rear_steering_angle_ += std::atan(
      -KD_ * (lateral_deviation - desired_lateral_deviation) / 4 +
      KD2_ * (course_deviation - desired_course_deviation) / KD_);
  } else {
    double alpha = 1 - curvature * (lateral_deviation - desired_lateral_deviation);
    double delta =
      KD_ * KD_ / alpha - 4 * curvature * KD2_ * (course_deviation - desired_course_deviation);
    rear_steering_angle_ += std::atan((KD_ - std::sqrt(delta)) / (2 * curvature));
  }

  if (std::abs(rear_steering_angle_) > M_PI_4) {  // ???
    rear_steering_angle_ += std::copysign(M_PI_2, -rear_steering_angle_);
  }

  return -rear_steering_angle_;
}

}  // namespace romea::core
