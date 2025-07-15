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

#ifndef ROMEA_CORE_CONTROL__OBSERVER__SLIDING_OBSERVER_PICARD_SKID_LYAPUNOV_HPP_
#define ROMEA_CORE_CONTROL__OBSERVER__SLIDING_OBSERVER_PICARD_SKID_LYAPUNOV_HPP_

#include <romea_core_common/signal/FirstOrderButterworth.hpp>

// std
#include <iostream>

namespace romea::core
{

class SlidingObserversPicardSkidLyapunov
{
public:
  struct Parameters
  {
    double kex;
    double kiex;
    double key;
    double ketheta;
    double kbeta;
    double kdotthetap;
    double kdotsp;
  };

public:
  SlidingObserversPicardSkidLyapunov(double step_time, const Parameters & parameters);

  void update(
    double epsilon_x,
    double epsilon_y,
    double epsilon_theta,
    double longi_speed,
    double angular_speed);

  void reset();

  [[nodiscard]] double getBetaR() const;
  [[nodiscard]] double getDotThetaP() const;
  [[nodiscard]] double getDotEpsilonSP() const;
  [[nodiscard]] double getXEstime() const;
  [[nodiscard]] double getYEstime() const;
  [[nodiscard]] double getThetaEstime() const;
  [[nodiscard]] double getXR() const;
  [[nodiscard]] double getYR() const;    
  [[nodiscard]] double getThetaR() const;
private:
  static constexpr int end_counter_ = 10;

  Parameters params_;
  double step_time_;

  double beta_r_estime_ = 0;
  double dot_theta_p_estime_ = 0;
  double dot_epsilon_s_p_estime_ = 0;

  int counter_ = 0;
  bool is_initialized_ = false;

  double epsilon_x_estime_ = 0;
  double epsilon_y_estime_ = 0;
  double epsilon_theta_estime_ = 0;

  //dot à l'iteration n-1
  double dot_epsilon_x_estime_n1_ = 0;
  double dot_epsilon_y_estime_n1_ = 0;
  double dot_epsilon_theta_estime_n1_ = 0;
  double dot_beta_n1_ = 0;
  double dot_dot_vg_n1_ = 0;
  double dot_dot_epsilon_theta_p_n1_ = 0;

  // Output check
  double X_R= 0.0;
  double Y_R = 0.0;
  double Theta_R = 0.0;
};

}  // namespace romea::core

#endif
