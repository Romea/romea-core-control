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
#include <romea_core_common/math/Algorithm.hpp>

// local
#include "romea_core_control/command/FollowTrajectorySkidSliding.hpp"

namespace romea::core
{

double computeBacksteppingSkidSteering(
  double lateral_deviation,
  double course_deviation,
  double curvature,
  double linear_speed,
  double linear_speed_disturbance,
  double angular_speed_disturbance,
  double sliding_angle,
  double maximal_angular_speed,
  double desired_lateral_deviation,
  double gain_lateral_kp,
  double gain_course_kp,
  double & target_course)
{
  // This code comes from the section 2.2.6.1 of the Luc Desbos's thesis: "Contribution à
  // l'élaboration de lois de commandes génériques pour le suivi de trajectoire d'engins agricoles
  // soumis à des dynamiques fortes et incertaines"

  double alpha = 1 - curvature * lateral_deviation;

  double diff_deviation = lateral_deviation - desired_lateral_deviation;
  target_course = std::atan2(gain_lateral_kp * diff_deviation, alpha);

  // This situation should never occur but it can be represented by a scenario where the robot
  // try to follow a circular trajectory and the robot is at the center.
  if (std::abs(alpha) < 1e-4) {
    std::cerr << "ERROR: angular speed command: singularity of alpha == 0:"
              << " sending maximal angular speed\n";
    return std::copysign(maximal_angular_speed, lateral_deviation);
  }

  double course_disturb = course_deviation + sliding_angle;
  double sum_lin_speed = linear_speed + linear_speed_disturbance;

  double ang_speed_course = gain_course_kp * (course_disturb - target_course);
  ang_speed_course = std::copysign(ang_speed_course, linear_speed);  // negative if lin_speed < 0
  double ang_speed_curvature = curvature * sum_lin_speed * std::cos(course_disturb) / alpha;
  double ang_speed_command = ang_speed_course + ang_speed_curvature - angular_speed_disturbance;

  // saturate angular speed
  if (std::abs(ang_speed_command) > maximal_angular_speed) {
    ang_speed_command = std::copysign(maximal_angular_speed, ang_speed_command);
  }

  return ang_speed_command;
}

}  // namespace romea::core
