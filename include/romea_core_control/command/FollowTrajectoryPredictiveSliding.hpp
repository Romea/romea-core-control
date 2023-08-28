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


#ifndef ROMEA_CORE_CONTROL__COMMAND__FOLLOWTRAJECTORYPREDICTIVESLIDING_HPP_
#define ROMEA_CORE_CONTROL__COMMAND__FOLLOWTRAJECTORYPREDICTIVESLIDING_HPP_

// std
#include <utility>
#include <vector>

// romea
#include "../FrontRearData.hpp"

namespace romea
{


class FollowTrajectoryPredictiveSliding
{
public:
  struct Parameters
  {
    double front_kp;
    double rear_kp;
    int horizon;
    double a0;
    double a1;
    double b1;
    double b2;
  };

public:
  FollowTrajectoryPredictiveSliding(
    const double & wheel_base,
    const Parameters & parameters);


  FrontRearData computeSteeringAngles(
    const double & lateral_deviation,
    const double & course_deviation,
    const double & curvature,
    const double & future_curvature,
    const double & front_steering_angle,
    const double & rear_steering_angle,
    const double & front_sliding_angle,
    const double & rear_sliding_angle,
    const double & front_maximal_steering_angle,
    const double & rear_maximal_steering_angle,
    const double & desired_lateral_deviation,
    const double & desired_course_deviation,
    const double & future_desired_lateral_deviation);

  void setFrontKP(const double & kp);

private:
  double computeFrontSteeringAngle_(
    const double & lateral_deviation,
    const double & course_deviation,
    const double & curvature,
    const double & future_curvature,
    const double & front_steering_angle,
    const double & rear_steering_angle,
    const double & front_sliding_angle,
    const double & rear_sliding_angle,
    const double & desired_lateral_deviation,
    const double & future_desired_lateral_deviation);


  double computeRearSteeringAngle_(
    const double & lateral_deviation,
    const double & course_deviation,
    const double & curvature,
    const double & rear_sliding_angle);

  double commandPred_(const double & CommFutur);

  std::vector<double> reference_(
    const double & CommFutur,
    const double & alpha,
    const double & feinte1);

private:
  double wheelbase_;
  double KD_;
  double KP_;
  double KD2_;
  int horizon_;
  double A0_;
  double A1_;
  double B1_;
  double B2_;
  double DeltaContr_av;
  double DeltaM_ar;
  double DeltaM_av;
  double DeltaM_av2;
  double DeltaEcartAV;
};

}  // namespace romea

#endif  // ROMEA_CORE_CONTROL__COMMAND__FOLLOWTRAJECTORYPREDICTIVESLIDING_HPP_
