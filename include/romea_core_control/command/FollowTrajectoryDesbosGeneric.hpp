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

#ifndef ROMEA_CORE_CONTROL__COMMAND__FOLLOW_TRAJECTORY_GENERIC_HPP_
#define ROMEA_CORE_CONTROL__COMMAND__FOLLOW_TRAJECTORY_GENERIC_HPP_

#include "romea_core_control/CommandsData.hpp"

namespace romea::core
{

class FollowTrajectoryDesbosGeneric
{
public:
  struct Parameters
  {
    double kp;
    double kd;
    double ks;
    bool adaptive_gains;
  };

public:
  FollowTrajectoryDesbosGeneric(double sampling_period, const Parameters & parameters);

  void reset();

  void set_desired_lateral_deviation(double desired_lat_dev);

  [[nodiscard]] double get_desired_lateral_deviation() const { return desired_lat_dev_; }
  [[nodiscard]] double get_kp() const { return kp_; }
  [[nodiscard]] double get_kd() const { return kd_; }
  [[nodiscard]] bool is_adaptive() const { return adaptive_gains_; }

  void set_gains(double kp, double kd, double ks);

  double compute_angular_speed(
    const double & lateral_deviation,
    const double & course_deviation,
    const double & maximal_angular_speed,
    const double & courbure,
    const double & future_courbure,
    const double & speed,
    const double & longitudinal_speed_command,
    // generic slip
    const double & lateral_slip,
    const double & angular_slip,
    // skid slip
    const double & speed_slip,
    const double & beta,
    const double & angular_skid_slip,
    double & omega_d,
    double & theta_error,
    double & tau,
    double & osc_eta,
    double & osc_amp);

  GenericCommandsData compute_commands(
    const double & lateral_deviation,
    const double & course_deviation,
    const double & longitudinal_deviation,
    const double & desired_speed,
    const double & maximal_angular_speed,
    const double & courbure,
    const double & future_courbure,
    const double & speed,
    const double & longitudinal_speed_command,
    // generic slip
    const double & lateral_slip,
    const double & angular_slip,
    const double & longitudinal_slip,
    // skid slip
    const double & speed_slip,
    const double & beta,
    const double & angular_skid_slip,
    double & omega_d,
    double & theta_error,
    double & tau,
    double & osc_eta,
    double & osc_amp);

  double compute_velocity(
    const double & lateral_deviation,
    const double & course_deviation,
    const double & longitudinal_deviation,
    const double & desired_speed,
    const double & courbure,
    const double & longitudinal_slip,
    const double & speed_slip,
    const double & beta);

private:
  double sampling_period_;
  double kp_;
  double kd_;
  double ks_;
  double desired_lat_dev_ = 0;
  double integrated_omega_ = 0;
  bool adaptive_gains_;
};

}  // namespace romea::core

#endif
