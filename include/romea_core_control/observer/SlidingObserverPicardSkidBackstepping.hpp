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

#ifndef ROMEA_CORE_CONTROL__OBSERVER__SLIDING_OBSERVER_PICARD_SKID_BACKSTEPPING_HPP_
#define ROMEA_CORE_CONTROL__OBSERVER__SLIDING_OBSERVER_PICARD_SKID_BACKSTEPPING_HPP_

#include <romea_core_common/signal/FirstOrderButterworth.hpp>

namespace romea::core
{

class SlidingObserversPicardSkidBackstepping
{
public:
  struct Parameters
  {
    double ky;
    double k_theta;
    double ks;
    double kis;

    double weight_slip_angle;
    double weight_linear_speed_disturb;
    double weight_angular_speed_disturb;
  };

public:
  SlidingObserversPicardSkidBackstepping(double step_time, const Parameters & parameters);

  void update(
    double epsilon_y,
    double epsilon_theta,
    double curvature,
    double longi_speed,
    double angular_speed,
    double curv_abscissa);

  void reset();


  // VectorXd getEstime();
  // VectorXd getAllEstime();

  [[nodiscard]] double getBetaR() const;
  [[nodiscard]] double getDotThetaP() const;
  [[nodiscard]] double getDotEpsilonSP() const;

private:
  Parameters params_;
  double step_time_;

  double beta_r_estime_ = 0;
  double dot_theta_p_estime_ = 0;
  double dot_epsilon_s_p_estime_ = 0;
  double epsilon_s_estime_ = 0;

  double integral = 0;

  romea::core::FirstOrderButterworth beta_r_estime_f_;
  romea::core::FirstOrderButterworth dot_theta_p_estime_f_;
  romea::core::FirstOrderButterworth dot_epsilon_s_p_estime_f_;

  int counter_ = 0;
  bool is_initialized_ = false;

private:
  double epsilon_s = 0;
  romea::core::FirstOrderButterworth epsilon_s_point_f_;
  double epsilon_s_old_ = 0;

  double epsilon_y_estime_ = 0;

  double epsilon_theta_estime_ = 0;
  romea::core::FirstOrderButterworth epsilon_theta_point_f_;
  double epsilon_theta_old_ = 0;

  // dot values at the previous iteration
  double dot_epsilon_s_estime_n1_ = 0;
  double dot_epsilon_y_estime_n1_ = 0;
  double dot_epsilon_theta_estime_n1_ = 0;
};

}  // namespace romea::core

#endif
