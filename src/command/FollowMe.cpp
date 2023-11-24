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
#include "romea_core_common/math/Algorithm.hpp"

// local
#include "romea_core_control/command/FollowMe.hpp"


namespace
{
const double SKID_STEERING_MAXIMAL_OMEGA_D = std::tan(50 * M_PI / 180);
const double SKID_STEERING_MAXIMAL_INTEDRATED_OMEGA_D = 0.75;
}

namespace romea
{
namespace core
{


//-----------------------------------------------------------------------------
FollowMe::FollowMe(
  const double & sampling_period,
  const Parameters & parameters)
: sampling_period_(sampling_period),
  kp_(parameters.kp),
  ki_(parameters.ki),
  kd_(parameters.kd),
  kdd_(parameters.kdd),
  desired_lat_dev_(0),
  integrated_omega_(0)
{
}

//-----------------------------------------------------------------------------
void FollowMe::reset()
{
  integrated_omega_ = 0;
}

//-----------------------------------------------------------------------------
void FollowMe::setDesiredLateralDeviation(const double & desired_lat_dev)
{
  if (std::abs(desired_lat_dev - desired_lat_dev_) > 0.1) {
    desired_lat_dev_ = desired_lat_dev;
    integrated_omega_ = 0;
  }
}

//-----------------------------------------------------------------------------
const double & FollowMe::getDesiredLateralDeviation()const
{
  return desired_lat_dev_;
}

//-----------------------------------------------------------------------------
double FollowMe::computeAngularSpeed(
  const double & lateral_deviation,
  const double & course_deviation,
  const double & maximal_angular_speed,
  double & omega_d,
  double & theta_error)
{
  omega_d = std::atan2(kp_ * (lateral_deviation - desired_lat_dev_), 1) - theta_error;

  if (std::abs(omega_d) > SKID_STEERING_MAXIMAL_OMEGA_D) {
    omega_d = copysign(SKID_STEERING_MAXIMAL_OMEGA_D, omega_d);
  }

  integrated_omega_ += sampling_period_ * (std::tan(course_deviation) - omega_d);

  if (std::abs(integrated_omega_) > SKID_STEERING_MAXIMAL_INTEDRATED_OMEGA_D) {
    integrated_omega_ = copysign(SKID_STEERING_MAXIMAL_INTEDRATED_OMEGA_D, integrated_omega_);
  }

  //  if(fabs(linear_speed) < 0.1)
  //    integrated_omega_ = 0;

  std::cout << kd_ * (std::tan(course_deviation) - omega_d) << " " << ki_ * integrated_omega_ <<
    std::endl;
  double angular_speed_command = kd_ * ((course_deviation) - omega_d) + ki_ * integrated_omega_;

  std::cout << " kp_" << kp_;
  std::cout << " ki_" << ki_;
  std::cout << " kd_" << kd_;
  std::cout << " lateral_deviation " << lateral_deviation;
  std::cout << " course_deviation " << course_deviation * 180 / M_PI;
  std::cout << " omega_d " << omega_d;
  std::cout << " integrated_omega_d " << integrated_omega_;
  std::cout << " angular_speed " << angular_speed_command << std::endl;

  if (std::abs(angular_speed_command) > maximal_angular_speed) {
    angular_speed_command = std::copysign(maximal_angular_speed, angular_speed_command);
  }

  std::cout << " angular_speed " << angular_speed_command << std::endl;

  return angular_speed_command;
}

//-----------------------------------------------------------------------------
FrontRearData FollowMe::computeSteeringAngles(
  const double & wheelbase,
  const double & lateral_deviation,
  const double & course_deviation,
  const double & rear_steering_angle,
  const double & maximal_front_steering_angle,
  const double & maximal_rear_steering_angle,
  double & omega_d,
  double & theta_consigne,
  const double & vitesse,
  const double & yaw_rate_leader,
  const double & desired_longitudinal_distance)
{
  assert(std::abs(kp_) > 0);
  assert(std::abs(kd_) > 0);
  assert(std::abs(kdd_) > 0);

  double VitMin_ = 0.7;
  omega_d = std::atan(kp_ * (lateral_deviation - desired_lat_dev_));


  double omega_d_max = 65 * M_PI / 180;
  if (fabs(omega_d) > (omega_d_max)) {
    omega_d = copysign((omega_d_max), omega_d);
  }

  double Thet2 = course_deviation + rear_steering_angle;
  double EpsThet = -(Thet2 - omega_d);
  double CoefYawrate = 0;
  if (fabs(desired_longitudinal_distance) < 0.2) {
    CoefYawrate = 0;
  }

  CoefYawrate = 1;

  double front_steering_angle_command =
    std::atan(
    wheelbase * (kd_ * EpsThet * std::cos(
      0 * Thet2) + CoefYawrate * yaw_rate_leader) / (VitMin_ * std::cos(
      rear_steering_angle)) + std::tan(rear_steering_angle));

  if (std::fabs(vitesse) > VitMin_) {
    // front_steering_angle_command =
    //   std::atan(
    //   wheelbase * (kd_ * EpsThet * std::cos(0 * Thet2) + CoefYawrate * yaw_rate_leader) /
    //   ((1 + CoefYawrate * (vitesse - 1)) * std::cos(rear_steering_angle)) +
    //   std::tan(rear_steering_angle));

    front_steering_angle_command = std::atan(
      wheelbase * (kd_ * EpsThet * std::cos(0 * Thet2) + CoefYawrate * yaw_rate_leader) /
      (vitesse * std::cos(rear_steering_angle)) + std::tan(rear_steering_angle));
  }

  if (std::abs(front_steering_angle_command) > maximal_front_steering_angle) {
    front_steering_angle_command = copysign(
      maximal_front_steering_angle,
      front_steering_angle_command);
  }

  theta_consigne = 0;
  if (std::abs(omega_d) > 5 * M_PI / 180) {
    theta_consigne = omega_d;
  }

  double ThetaError2 = theta_consigne - course_deviation;

  //  integrated_omega_ += sampling_period_*(tan(course_deviation)-omega_d);
  //  if (fabs(integrated_omega_) > 0.75)
  //    integrated_omega_ = copysign(0.75, integrated_omega_);

  //  if(fabs(linear_speed) < 0.1)
  //    integrated_omega_ = 0;

  //  double kdd =1.2;//1.8

  double rear_steering_angle_command = -course_deviation - (1 / kd_) *
    (kdd_ * ThetaError2 - kd_ * omega_d);

  if (std::abs(rear_steering_angle_command) > maximal_rear_steering_angle) {
    rear_steering_angle_command = std::copysign(
      maximal_rear_steering_angle,
      rear_steering_angle_command);
  }

  std::cout << " kp_" << kp_;
  std::cout << " ki_" << ki_;
  std::cout << " kd_" << kd_;
  std::cout << " lateral_deviation " << lateral_deviation;
  std::cout << " course_deviation " << course_deviation;
  std::cout << " omega_d " << omega_d;
  std::cout << " integrated_omega_ " << integrated_omega_;
  std::cout << " theta_consigne " << theta_consigne;
  std::cout << " front_steering_angle_command " << front_steering_angle_command;
  std::cout << " rear_steering_angle_command " << rear_steering_angle_command << std::endl;

  rear_steering_angle_command = 0;
  return {front_steering_angle_command, rear_steering_angle_command};
}


//-----------------------------------------------------------------------------
FrontRearData FollowMe::computeSteeringAngles(
  const double & wheelbase,
  const double & lateral_deviation,
  const double & course_deviation,
  const double & curvature,
  const double & /*speed*/,
  const double & rear_steering_angle,
  const double & rear_sliding_angle,
  const double & front_sliding_angle,
  const double & minimal_theta,
  const double & maximal_theta,
  const double & maximal_front_steering_angle,
  const double & maximal_rear_steering_angle,
  double & omega_d,
  double & theta_consigne)
{
  //    std::cout << "Gain command kp ********" << kp_ << "** kd **" << kd_ << std::endl;
  // A basee vitesse Kp=-.5, kd=1, kpp=1.5
  // kp_=-0.25;
  // kd_=1.75;
  // double kdd_=3;

  //    kp_=-0.15;
  //    kd_=.8;
  //    double kdd_=.4;

  assert(std::abs(kp_) > 0);
  assert(std::abs(kd_) > 0);
  assert(std::abs(kdd_) > 0);

  std::cout << " lateral deviation " << lateral_deviation << std::endl;
  std::cout << " desired_lat_dev_ " << desired_lat_dev_ << std::endl;

  //  yd =1.25;
  double alpha = 1 - curvature * (lateral_deviation + 1.35);
  //  omega_d = kp_*(lateral_deviation-desired_lat_dev_);
  omega_d = atan(kp_ * (lateral_deviation - desired_lat_dev_) / alpha);

  std::cout << "new alpha " << alpha << " omega_d" << omega_d << std::endl;


  if (fabs(omega_d) > (40 * M_PI / 180)) {
    omega_d = copysign((40 * M_PI / 180), omega_d);
  }

  if (omega_d < minimal_theta) {
    omega_d = minimal_theta;
  }

  if (omega_d > maximal_theta) {
    omega_d = maximal_theta;
  }

  double Thet2 = course_deviation + rear_steering_angle + rear_sliding_angle;
  double EpsThet = -(Thet2 - omega_d);

  // braq_F = std::atan(
  // kd_ * EpsThet * wheelbase * cos(Thet2) / cos(rear_steering_angle) + tan(rear_steering_angle));
  double braq_F = std::atan(
    (kd_ * EpsThet + curvature) * wheelbase * std::cos(Thet2) /
    cos(rear_steering_angle + rear_sliding_angle) +
    std::tan(rear_steering_angle + rear_sliding_angle)) - front_sliding_angle;

  theta_consigne = 0;
  if ((fabs(omega_d) > 5 * M_PI / 180) || (fabs(EpsThet) > 7.5 * M_PI / 180)) {
    theta_consigne = omega_d;
  }

  double ThetaError2 = theta_consigne - course_deviation;
  //  integrated_omega_ += 0.1*(tan(course_deviation)-omega_d);
  //  if (fabs(integrated_omega_) > 0.75)
  //    integrated_omega_ = copysign(0.75, integrated_omega_);

  //  if(fabs(speed) < 0.1)
  //    integrated_omega_ = 0;

  //  braq_R=-course_deviation-(1/kd_)*( kdd_*ThetaError2 - kd_*omega_d);
  double braq_R = -course_deviation - rear_sliding_angle - (1 / kd_) *
    (kdd_ * ThetaError2 - kd_ * omega_d);

  std::cout << " new command " << omega_d << " " << Thet2 << " " << EpsThet << " " << braq_F <<
    " " << theta_consigne << " " << ThetaError2 << " " << braq_R << std::endl;

  //    braq_R=0;
  if (std::abs(braq_R) > maximal_rear_steering_angle) {
    braq_R = copysign(maximal_rear_steering_angle, braq_R);
  }

  if (std::abs(braq_F) > maximal_front_steering_angle) {
    braq_F = copysign(maximal_front_steering_angle, braq_F);
  }

  return {braq_F, braq_R};
}

//-----------------------------------------------------------------------------
FrontRearData FollowMe::computeSteeringAngles(
  const double & wheelbase,
  const double & lateral_deviation,
  const double & course_deviation,
  const double & curvature,
  const double & /*speed*/,
  const double & rear_steering_angle,
  const double & rear_sliding_angle,
  const double & front_sliding_angle,
  const double & minimal_theta,
  const double & maximal_theta,
  const double & front_maximal_steering_angle,
  const double & rear_maximal_steering_angle,
  const double & courbe0,
  const double & courbe1,
  const double & courbe2,
  const double & lambda,
  double & omega_d,
  double & theta_consigne)
{
  assert(std::abs(kp_) > 0);
  assert(std::abs(kd_) > 0);
  assert(std::abs(kdd_) > 0);

  //  // Essais MecaFutur
  //  kp_=-0.15;
  //  kd_=0.3;
  //  double kdd_=0.6;
  //  double empat=1.2;


  //    int nH = 20;
  //    double coef=75;
  int nH = 30;
  //    double coef=0.5;
  double coef = 0.92;
  double yd = 1.25;

  double gamma = 0;
  double Sigma2 = 0;
  double Sigma3 = 0;
  double Sigma = 0;
  double SigmaExp = 0;
  double Te = 0.10;


  double alpha = 1 - curvature * (lateral_deviation + yd);

  for (int i = 0; i < nH; i++) {
    gamma += lateral_deviation * (std::exp(-coef * i * Te)) * i * Te;
    Sigma3 += i * Te * i * Te * i * Te;
    Sigma2 += i * Te * i * Te;
    Sigma += i * Te;
    SigmaExp += i * Te * std::exp(-coef * i * Te);
  }

  // Ancienne version commande Pred
  // std::cout << " gamma = " << gamma << "  Ecart   = " << lateral_deviation << std::endl;
  // double commande_inter = (-gamma + 0*courbe0*Sigma)/Sigma2  + 1*courbe1;
  // std::cout << " commande_inter = " << commande_inter << "A =  " << courbe1 << std::endl;
  // if (commande_inter>0.9)
  //   commande_inter=std::copysign(0.9,commande_inter);

  // std::cout << " SigmaExp = " << SigmaExp << "  Ecart   = " << lateral_deviation << std::endl;
  double commande_inter =
    ((-lateral_deviation - 0 * courbe0) * Sigma + lateral_deviation * SigmaExp - 0 * courbe2 *
    Sigma3) / Sigma2 + 1 * courbe1;
  // std::cout << " commande_inter = " << commande_inter << "A =  " << courbe1 << std::endl;

  if (commande_inter > 0.9) {
    commande_inter = std::copysign(0.9, commande_inter);
  }


  omega_d = std::asin(commande_inter);
  std::cout << " ******************************* commande_inter = " << commande_inter << std::endl;
  omega_d = lambda * omega_d + (1 - lambda) * atan(
    kp_ * (lateral_deviation - desired_lat_dev_) / alpha);


  if (fabs(omega_d) > (45 * M_PI / 180)) {
    omega_d = copysign((45 * M_PI / 180), omega_d);
  }

  if (omega_d < minimal_theta) {
    omega_d = minimal_theta;
  }
  if (omega_d > maximal_theta) {
    omega_d = maximal_theta;
  }


  double Thet2 = course_deviation + rear_steering_angle + rear_sliding_angle;
  double EpsThet = -(Thet2 - omega_d);

  //  braq_F  = atan(kd_*EpsThet*empat*cos(Thet2)/cos(deltaR) + tan(deltaR));
  double braq_F = std::atan(
    (kd_ * EpsThet + (curvature)) * wheelbase * cos(Thet2) /
    cos(rear_steering_angle + rear_sliding_angle) +
    tan(rear_steering_angle + rear_sliding_angle)) - front_sliding_angle;


  //  if(fabs(braq_F) > (20*M_PI/180))
  //    braq_F = copysign((20*M_PI/180), braq_F);

  theta_consigne = 0;
  if ((fabs(omega_d) > 5 * M_PI / 180) || (fabs(EpsThet) > 7.5 * M_PI / 180)) {
    theta_consigne = omega_d;
  }


  double ThetaError2 = theta_consigne - course_deviation;
  //  integrated_omega_ += 0.1*(tan(course_deviation)-omega_d);
  //  if (fabs(integrated_omega_) > 0.75)
  //    integrated_omega_ = copysign(0.75, integrated_omega_);

  //  if(fabs(speed) < 0.1)
  //    integrated_omega_ = 0;
  //  double a1=0;
  //  braq_R=-course_deviation-(1/kd_)*( kdd_*ThetaError2 - kd_*omega_d);
  double braq_R = -course_deviation - rear_sliding_angle - (1 / kd_) *
    (kdd_ * ThetaError2 - kd_ * omega_d);


  //    braq_R=0;
  if (std::abs(braq_R) > (rear_maximal_steering_angle * M_PI / 180)) {
    braq_R = std::copysign((rear_maximal_steering_angle * M_PI / 180), braq_R);
  }

  if (std::abs(braq_F) > (front_maximal_steering_angle * M_PI / 180)) {
    braq_F = std::copysign((front_maximal_steering_angle * M_PI / 180), braq_F);
  }
  // return a1;
  return {braq_F, braq_R};
}

}  // namespace core
}  // namespace romea
