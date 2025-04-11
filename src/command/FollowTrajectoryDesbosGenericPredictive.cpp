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

#include <Eigen/Dense>
#include <cmath>

// romea core
#include <romea_core_common/math/Algorithm.hpp>

// local
#include "romea_core_control/command/FollowTrajectoryDesbosGenericPredictive.hpp"

constexpr double SKID_STEERING_MAXIMAL_OMEGA_D = 45. * M_PI / 180.;

namespace romea::core
{

//-----------------------------------------------------------------------------
FollowTrajectoryDesbosGenericPredictive::FollowTrajectoryDesbosGenericPredictive(
  double sampling_period, const Parameters & parameters)
: sampling_period_(sampling_period),
  kp_(parameters.kp),
  kd_(parameters.kd),
  ks_(parameters.ks),
  alpha_(parameters.alpha),
  a0_(parameters.a0),
  a1_(parameters.a1),
  b1_(parameters.b1),
  b2_(parameters.b2),
  adaptive_gains_(parameters.adaptive_gains),
  lmpc_(parameters.lmpc),
  horizon_(parameters.horizon),
  model_order_(parameters.model_order)
{
}

//-----------------------------------------------------------------------------
void FollowTrajectoryDesbosGenericPredictive::reset()
{
  integrated_omega_ = 0;
}

//-----------------------------------------------------------------------------
void FollowTrajectoryDesbosGenericPredictive::set_desired_lateral_deviation(double desired_lat_dev)
{
  if (std::abs(desired_lat_dev - desired_lat_dev_) > 0.1) {
    desired_lat_dev_ = desired_lat_dev;
    integrated_omega_ = 0;
  }
}

void FollowTrajectoryDesbosGenericPredictive::set_gains(double kp, double kd, double ks)
{
  kp_ = kp;
  kd_ = kd;
  ks_ = ks;
}


//-----------------------------------------------------------------------------
double FollowTrajectoryDesbosGenericPredictive::compute_angular_speed(
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
  double dt)
{
  if (lmpc_) {
    std::cout << "model_order_ : " << model_order_ << std::endl;
    if (model_order_ == 2) {
      return compute_angular_speed_lmpc(
        lateral_deviation,
        course_deviation,
        curvature,
        future_curvature,
        speed,
        longitudinal_speed_command,
        angular_speed,
        lateral_slip,
        angular_slip,
        courbe1,
        courbe2,
        lambda,
        omega_d,
        theta_consigne,
        tau,
        dt);
    }
    if (model_order_ == 1) {
      return compute_angular_speed_lmpc1(
        lateral_deviation,
        course_deviation,
        curvature,
        future_curvature,
        speed,
        longitudinal_speed_command,
        angular_speed,
        lateral_slip,
        angular_slip,
        courbe1,
        courbe2,
        lambda,
        omega_d,
        theta_consigne,
        curv_pred,
        tau,
        dt);
    }
  }
  return compute_angular_speed_hmpc(
    lateral_deviation,
    course_deviation,
    curvature,
    future_curvature,
    speed,
    lateral_slip,
    angular_slip,
    courbe1,
    courbe2,
    lambda,
    omega_d,
    theta_consigne,
    tau);
}

//-----------------------------------------------------------------------------
double FollowTrajectoryDesbosGenericPredictive::compute_angular_speed_hmpc(
  double lateral_deviation,
  double course_deviation,
  double curvature,
  double  /*future_curvature*/,
  double speed,
  double lateral_slip,
  double angular_slip,
  double courbe1,
  double courbe2,
  double  /*lambda*/,
  double & omega_d,
  double &  /*theta_consigne*/,
  double  /*tau*/)
{
  assert(kp_ < 0);
  assert(kd_ < 0);

  double Sigma2 = 0;
  double Sigma3 = 0;
  double Sigma = 0;
  double SigmaExp = 0;
  double Te = 0.02;
  int nH = horizon_ / Te;

  double alpha = 1 - curvature * lateral_deviation;

  for (int i = 0; i < nH; i++) {
    Sigma3 += i * Te * i * Te * i * Te;
    Sigma2 += i * Te * i * Te;
    Sigma += i * Te;
    SigmaExp += i * Te * std::exp(kp_ * i * Te);
  }

  double commande_inter =
    ((lateral_deviation * (Sigma - SigmaExp) - courbe1 * Sigma2 - courbe2 * Sigma3) / Sigma2 -
     lateral_slip) /
    alpha;

  omega_d = -atan2(commande_inter, 1);

  // VERSION PAPIER
  // double error = tan(course_deviation) - tan(omega_d);
  // double angular_speekd_d_command = kd_*error*pow(cos(course_deviation),2) + curvature*speed*cos(course_deviation)/alpha - angular_slip;

  // VERSION LUC
  double error = atan2(sin(course_deviation - omega_d), cos(course_deviation - omega_d));
  double angular_speed_command =
    kd_ * error + curvature * speed * cos(course_deviation) / alpha - angular_slip;

  return angular_speed_command;
}

//-----------------------------------------------------------------------------
double FollowTrajectoryDesbosGenericPredictive::compute_angular_speed_lmpc(
  double lateral_deviation,
  double course_deviation,
  double curvature,
  double future_curvature,
  double speed,
  double longitudinal_speed_command,
  double angular_speed,
  double lateral_slip,
  double angular_slip,
  double  /*courbe1*/,
  double  /*courbe2*/,
  double  /*lambda*/,
  double & omega_d,
  double & pred_command,
  double  /*tau*/,
  double  /*dt*/)
{
  assert(std::abs(kp_) > 0);
  assert(std::abs(kd_) > 0);

  double activ_obs = 0;
  if (fabs(future_curvature) > 0.1) {
    activ_obs = 1;
  }

  omega_d = std::atan2(
    kp_ * (lateral_deviation - desired_lat_dev_) - lateral_slip * activ_obs,
    1 - curvature * lateral_deviation);

  if (std::abs(omega_d) > SKID_STEERING_MAXIMAL_OMEGA_D) {
    omega_d = copysign(SKID_STEERING_MAXIMAL_OMEGA_D, omega_d);
  }

  // integrated_omega_ += sampling_period_*(std::tan(course_deviation)-omega_d);

  // if (std::abs(integrated_omega_) > SKID_STEERING_MAXIMAL_INTEDRATED_OMEGA_D)
  // {
  //   integrated_omega_ = copysign(SKID_STEERING_MAXIMAL_INTEDRATED_OMEGA_D, integrated_omega_);
  // }

  //  if(fabs(linear_speed) < 0.1)
  //    integrated_omega_ = 0;

  double error = atan2(sin(course_deviation - omega_d), cos(course_deviation - omega_d));
  // std::cout << kd_*(std::tan(course_deviation)-omega_d) <<" "<< ki_*integrated_omega_ << std::endl;

  std::cout << " future_curvature:" << future_curvature << std::endl;
  std::cout << " speed:" << speed << std::endl;
  pred_command = command_pred(
    longitudinal_speed_command * future_curvature * cos(course_deviation) /
      (1 - curvature * lateral_deviation),
    angular_speed);
  double angular_speed_command = kd_ * (error) + pred_command - angular_slip * activ_obs;

  angular_speed_command_2_ = angular_speed_command;
  return angular_speed_command;
}

//-----------------------------------------------------------------------------
double FollowTrajectoryDesbosGenericPredictive::compute_angular_speed_lmpc1(
  double lateral_deviation,
  double course_deviation,
  double curvature,
  double future_curvature,
  double speed,
  double longitudinal_speed_command,
  double  /*angular_speed*/,
  double lateral_slip,
  double angular_slip,
  double  /*courbe1*/,
  double  /*courbe2*/,
  double  /*lambda*/,
  double & omega_d,
  double & pred_command,
  double & curv_pred,
  double tau,
  double dt)
{
  if (adaptive_gains_) {
    kd_ = kd_ + 1. / 5. * (-1. / (1.0 * tau) - kd_);
    kd_ = std::min(-0.02, std::max(-5., kd_));
  }

  double N = 3;  //std::min( std::max(2*std::abs(lateral_deviation)+3, 3.),4. );
  kp_ = kd_ / (N * longitudinal_speed_command);  // HARD CODED COMMANDED SPEED (0.5)
  kp_ = std::min(-0.02, std::max(-4., kp_));
  std::cout << "LMPC1" << std::endl;
  double activ_obs = 0;

  if (fabs(future_curvature) > 0.1) {
    activ_obs = 0;
  }

  //  omega_d = std::atan2(kp_*(lateral_deviation-desired_lat_dev_)-lateral_slip*activ_obs,1-courbure*lateral_deviation);

  omega_d = std::atan2(
    kp_ * (lateral_deviation - desired_lat_dev_) - lateral_slip * activ_obs,
    1 - curvature * lateral_deviation);

  if (std::abs(omega_d) > SKID_STEERING_MAXIMAL_OMEGA_D) {
    omega_d = copysign(SKID_STEERING_MAXIMAL_OMEGA_D, omega_d);
  }

  double error = atan2(sin(course_deviation - omega_d), cos(course_deviation - omega_d));

  std::cout << " future_curvature:" << future_curvature << std::endl;
  std::cout << " speed:" << speed << std::endl;
  curv_pred = longitudinal_speed_command * future_curvature * cos(course_deviation) /
              (1 - curvature * lateral_deviation);

  // angular_speed_command_2_ = angular_speed - angular_speed_command_2_;
  if (tau < 0.1) {
    pred_command = command_pred1(curv_pred, angular_speed_command_2_, 0.8, dt);
  } else {
    pred_command = command_pred1(curv_pred, angular_speed_command_2_, tau, dt);
  }
  double angular_speed_command = kd_ * (error) + pred_command - angular_slip * activ_obs;
  angular_speed_command_2_ = angular_speed_first_order_;
  // angular_speed_command_2_ = kd_*(error);
  if (dt < 0.2) {
    if (tau < 0.1) {
      angular_speed_first_order_ =
        angular_speed_first_order_ + dt * (pred_command - angular_speed_first_order_) / 0.8;
    } else {
      angular_speed_first_order_ =
        angular_speed_first_order_ + dt * (pred_command - angular_speed_first_order_) / tau;
    }
  }
  return angular_speed_command;
}

//-----------------------------------------------------------------------------
GenericCommandsData FollowTrajectoryDesbosGenericPredictive::compute_commands(
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
  double  /*angular_skid_slip*/,
  // approx traj
  double courbe1,
  double courbe2,
  double  /*lambda*/,
  double & omega_d,
  double & theta_consigne,
  double tau,
  double dt)
{
  double bypass;
  double angular_speed_command = compute_angular_speed(
    lateral_deviation,
    course_deviation,
    curvature,
    future_curvature,
    speed,
    longitudinal_speed_command,
    angular_speed,
    lateral_slip,
    angular_slip,
    courbe1,
    courbe2,
    0,
    omega_d,
    theta_consigne,
    bypass,
    tau,
    dt);

  double longitudinal_speed_command_guiding = compute_velocity(
    lateral_deviation,
    course_deviation,
    longitudinal_deviation,
    desired_speed,
    curvature,
    longitudinal_slip,
    speed_slip,
    beta);

  return {longitudinal_speed_command_guiding, angular_speed_command};
}

//-----------------------------------------------------------------------------
double FollowTrajectoryDesbosGenericPredictive::compute_velocity(
  double lateral_deviation,
  double course_deviation,
  double longitudinal_deviation,
  double desired_speed,
  double courbure,
  double longitudinal_slip,
  double speed_slip,
  double beta)
{
  return (ks_ * longitudinal_deviation + desired_speed - longitudinal_slip) *
           (1 - courbure * lateral_deviation) / cos(course_deviation + beta) -
         speed_slip;
}

//------------------------------------------------------------------------------
double FollowTrajectoryDesbosGenericPredictive::command_pred1(
  double CommFutur, double angular_speed, double tau, double  /*dt*/)
{
  double u;
  double DT = 0.1, TAU = tau;
  double R1 = 0., R2 = 0., Ai = 0., di = 0.;

  std::vector<double> Ref(horizon_ + 3);
  std::cout << "LMPC Model Order 1" << std::endl;
  Ref = reference(CommFutur, alpha_, angular_speed);

  for (int i = 1; i < (horizon_ + 1); i++) {
    di = Ref[i] - pow((1 - DT / TAU), i + 1) * angular_speed;
    Ai = 0.;
    for (int j = 0; j < (i + 1); j++) {
      Ai = Ai + pow((1 - DT / TAU), i - j) * DT / TAU;
    }
    if (CommFutur > 0.25) {
      std::cout << "Ref[5]:" << Ref[horizon_ + 1] << "  Ai:" << Ai << "  di:" << di << std::endl;
    }
    R2 = R2 + Ai * di;
    R1 = R1 + (Ai * Ai);
  }
  u = R2 / R1;
  return u;
}

//------------------------------------------------------------------------------
double FollowTrajectoryDesbosGenericPredictive::command_pred(double CommFutur, double angular_speed)
{
  double feinte, delta;

  Eigen::Matrix3d Fm, Ident, mult2;
  Eigen::Vector3d Gm, Etat;
  Eigen::RowVector3d mult, Cm;
  double CommFut, R1 = 0, R2 = 0, Ai, di;

  //------------------------------------------------
  // Modele interne
  //------------------------------------------------
  Fm(0, 0) = b1_;
  Fm(0, 1) = b2_;
  Fm(0, 2) = a1_;
  Fm(1, 0) = 1;
  Fm(1, 1) = 0;
  Fm(1, 2) = 0;
  Fm(2, 0) = 0;
  Fm(2, 1) = 0;
  Fm(2, 2) = 0;

  Cm(0, 0) = 1;
  Cm(0, 1) = 0;
  Cm(0, 2) = 0;

  Gm(0, 0) = a0_;
  Gm(1, 0) = 0;
  Gm(2, 0) = 1;
  //------------------------------------------------

  Ident = Eigen::Matrix3d::Identity();
  mult2 = Eigen::Matrix3d::Zero();

  std::cout << "LMPC Model Order 2" << std::endl;

  Etat(0, 0) = angular_speed;
  Etat(1, 0) = angular_speed_2_;
  Etat(2, 0) = angular_speed_command_2_;

  angular_speed_2_ = angular_speed;

  CommFut = CommFutur;
  feinte = angular_speed;

  mult(0, 0) = 1;
  mult(0, 1) = 0;
  mult(0, 2) = 0;

  std::vector<double> Ref(horizon_ + 3);
  // std::cout << " alpha_:" << alpha_ << std::endl;
  // std::cout << " CommFutur:" << CommFutur << std::endl;
  Ref = reference(CommFutur, alpha_, feinte);
  // std::cout << " Ref:" << Ref[horizon_+1];

  for (int i = 1; i < (horizon_ + 1); i++) {
    // Calcul intermediaire Ai
    mult2 = Fm;
    Ai = mult * Ident * Gm;

    // Calcul intermediaire di
    CommFut = Ref[i];
    di = CommFut - (Cm * mult2 * Etat);

    for (int j = 1; j < (i + 1); j++) {
      Ai = Ai + mult * mult2 * Gm;
      mult2 = mult2 * Fm;
    }
    // std::cout << " d" << i << ":" << di;

    R2 = R2 + Ai * di;
    R1 = R1 + (Ai * Ai);
  }
  delta = R2 / R1;
  // std::cout << " detlta:" << delta << std::endl;
  return delta;
}

//------------------------------------------------------------------------------
std::vector<double> FollowTrajectoryDesbosGenericPredictive::reference(
  double CommFutur, double alpha, double feinte1)
{
  std::vector<double> delta(horizon_ + 3);

  // delta[1] = feinte1;
  for (
    int j = 1; j < (horizon_ + 2);
    j++)  // indice de départ modifié au 22 mars (before j=2 now j=1) ne change absoluement rien normalement
  {
    delta[j] = CommFutur - (pow(alpha, j - 1)) * (CommFutur - feinte1);
  }

  return delta;
}
}  // namespace romea::core
