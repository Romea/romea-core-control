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

#ifndef ROMEA_CORE_CONTROL__COMMAND__FOLLOW_TRAJECTORY_GENERIC_PREDICTIVE_HPP_
#define ROMEA_CORE_CONTROL__COMMAND__FOLLOW_TRAJECTORY_GENERIC_PREDICTIVE_HPP_

// std
#include <vector>

// local
#include "romea_core_control/CommandsData.hpp"

namespace romea::core
{

class FollowTrajectoryDesbosGenericPredictive
{
public:
  struct Parameters
  {
    double kp;
    double kd;
    double ks;
    double alpha;
    double a0;
    double a1;
    double b1;
    double b2;
    bool adaptive_gains;
    bool lmpc;
    int horizon;
    int model_order;
  };

public:
  FollowTrajectoryDesbosGenericPredictive(double sampling_period, const Parameters & parameters);

  void reset();

  void set_desired_lateral_deviation(double desired_lat_dev);

  [[nodiscard]] double get_desired_lateral_deviation() const { return desired_lat_dev_; }
  [[nodiscard]] double get_kp() const { return kp_; }
  [[nodiscard]] double get_kd() const { return kd_; }
  [[nodiscard]] bool is_adaptive() const { return adaptive_gains_; }
  [[nodiscard]] double get_horizon() const { return horizon_; }

  void set_gains(double kp, double kd, double ks);

  double compute_angular_speed(
    double lateral_deviation,
    double course_deviation,
    double curvature,
    double future_curvature,
    double speed,
    double longitudinal_speed_command,
    double angular_speed,
    double lateral_slip,
    double angular_slip,
    double courbe1,
    double courbe2,
    double lambda,
    double & omega_d,
    double & theta_consigne,
    double & curv_pred,
    double tau,
    double dt);

  double compute_angular_speed_hmpc(
    double lateral_deviation,
    double course_deviation,
    double curvature,
    double future_curvature,
    double speed,
    double lateral_slip,
    double angular_slip,
    double courbe1,
    double courbe2,
    double lambda,
    double & omega_d,
    double & theta_consigne,
    double tau);

  double compute_angular_speed_lmpc(
    double lateral_deviation,
    double course_deviation,
    double curvature,
    double future_curvature,
    double speed,
    double longitudinal_speed_command,
    double angular_speed,
    double lateral_slip,
    double angular_slip,
    double courbe1,
    double courbe2,
    double lambda,
    double & omega_d,
    double & pred_command,
    double tau,
    double dt);

  double compute_angular_speed_lmpc1(
    double lateral_deviation,
    double course_deviation,
    double curvature,
    double future_curvature,
    double speed,
    double longitudinal_speed_command,
    double angular_speed,
    double lateral_slip,
    double angular_slip,
    double courbe1,
    double courbe2,
    double lambda,
    double & omega_d,
    double & pred_command,
    double & curv_pred,
    double tau,
    double dt);

  GenericCommandsData compute_commands(
    double lateral_deviation,
    double course_deviation,
    double longitudinal_deviation,
    double desired_speed,
    double curvature,
    double future_curvature,
    double speed,
    double longitudinal_speed_command,
    double angular_speed,
    // generic slip
    double lateral_slip,
    double angular_slip,
    double longitudinal_slip,
    // skid slip
    double speed_slip,
    double beta,
    double angular_skid_slip,
    // approx traj
    double courbe1,
    double courbe2,
    double lambda,
    double & omega_d,
    double & theta_consigne,
    double tau,
    double dt);

  double compute_velocity(
    double lateral_deviation,
    double course_deviation,
    double longitudinal_deviation,
    double desired_speed,
    double courbure,
    double longitudinal_slip,
    double speed_slip,
    double beta);

  double command_pred1(
    double CommFutur, double angular_speed, double tau, double dt);

  double command_pred(
    double CommFutur, double angular_speed);

  std::vector<double> reference(
    double CommFutur, double alpha, double feinte1);

private:
  double sampling_period_;
  double kp_;
  double kd_;
  double ks_;
  double alpha_;
  double a0_;
  double a1_;
  double b1_;
  double b2_;
  bool adaptive_gains_;
  bool lmpc_;
  int horizon_;
  int model_order_;
  double desired_lat_dev_ = 0;
  double integrated_omega_ = 0;
  double angular_speed_2_ = 0;
  double angular_speed_command_2_ = 0;
  double angular_speed_first_order_ = 0;
};

}  // namespace romea::core

#endif
