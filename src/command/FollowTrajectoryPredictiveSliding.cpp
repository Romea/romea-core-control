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
#include <iostream>

// romea
#include "romea_core_common/math/Algorithm.hpp"
#include "romea_core_control/command/FollowTrajectoryPredictiveSliding.hpp"


// namespace {
// const double DEFAULT_KD = 0.7;
// const double DEFAULT_KP =DEFAULT_KD*DEFAULT_KD/4;
// const double DEFAULT_KD2 = 0.7;

// const unsigned int HORIZON_PREDICTION = 10;
// const double A0=0.1642, A1=0.1072, B1=1.0086, B2=-0.2801;
// const double A0=0.0052, A1=0.0049, B1=1.8282, B2=-0.8383;

//}

namespace romea
{
namespace core
{


//-----------------------------------------------------------------------------
FollowTrajectoryPredictiveSliding::FollowTrajectoryPredictiveSliding(
  double whee_base,
  const Parameters & parameters)
: wheelbase_(whee_base),
  KD_(parameters.front_kp),
  KP_(KD_ * KD_ / 4.),
  KD2_(parameters.rear_kp),
  horizon_(parameters.horizon),
  A0_(parameters.a0),
  A1_(parameters.a1),
  B1_(parameters.b1),
  B2_(parameters.b2),
  DeltaContr_av(0),
  DeltaM_ar(0),
  DeltaM_av(0),
  DeltaM_av2(0),
  DeltaEcartAV(0)
{
}

//-----------------------------------------------------------------------------
void FollowTrajectoryPredictiveSliding::setFrontKP(double kp)
{
  KD_ = kp;
  KP_ = (KD_ * KD_ / 4.);
}

//-----------------------------------------------------------------------------
FrontRearData FollowTrajectoryPredictiveSliding::computeSteeringAngles(
  double lateral_deviation,
  double course_deviation,
  double curvature,
  double future_curvature,
  double front_steering_angle,
  double rear_steering_angle,
  double front_sliding_angle,
  double rear_sliding_angle,
  double front_maximal_steering_angle,
  double rear_maximal_steering_angle,
  double desired_lateral_deviation,
  double desired_course_deviation,
  double future_desired_lateral_deviation)
{
  // compute front steering angle
  double front_steering_angle_command = computeFrontSteeringAngle_(
    lateral_deviation,
    course_deviation,
    curvature,
    future_curvature,
    front_steering_angle,
    rear_steering_angle,
    -front_sliding_angle,
    rear_sliding_angle,
    desired_lateral_deviation,
    future_desired_lateral_deviation);


  front_steering_angle_command = clamp(
    front_steering_angle_command,
    -front_maximal_steering_angle,
    front_maximal_steering_angle);

  double rear_steering_angle_command = computeRearSteeringAngle_(
    lateral_deviation,
    course_deviation,
    curvature,
    rear_sliding_angle,
    desired_lateral_deviation,
    desired_course_deviation);

  rear_steering_angle_command = clamp(
    rear_steering_angle_command,
    -rear_maximal_steering_angle,
    rear_maximal_steering_angle);

  return {front_steering_angle_command, rear_steering_angle_command};
}

//-----------------------------------------------------------------------------
double FollowTrajectoryPredictiveSliding::computeFrontSteeringAngle_(
  double lateral_deviation,
  double course_deviation,
  double curvature,
  double future_curvature,
  double front_steering_angle,
  double rear_steering_angle,
  double front_sliding_angle,
  double rear_sliding_angle,
  double desired_lateral_deviation,
  double future_desired_lateral_deviation)
{
  DeltaM_av2 = DeltaM_av;
  DeltaM_av = front_steering_angle;
  DeltaM_ar = rear_steering_angle;

  // calculs intermediaires
  double v1 = course_deviation + rear_sliding_angle + DeltaM_ar;  // =thilde_2 in papers
  double v2 = 1 - curvature * lateral_deviation;  // =alpha in papers
  double v3 = -KD_ * v2 * std::tan(v1) - KP_ * (lateral_deviation - desired_lateral_deviation) +
    curvature * v2 * (std::tan(v1)) * (std::tan(v1));  // =A in papers

  // Commande Futur a Horizon H, def. par cRefPred
  // double  UHcomm  = (wheelbase_/std::cos(rear_sliding_angle+DeltaM_ar))*
  //   future_curvature*cos(v1)/v2;
  // to be checked
  double UHcomm = (wheelbase_ / std::cos(rear_sliding_angle + DeltaM_ar)) *
    (future_curvature / (1 - future_curvature * (future_desired_lateral_deviation)));

  // double  delta_traj_pred = atan(UHcomm);
  // double  delta_traj_pred = atan(UHcomm)-1*2*cRefPred*front_sliding_angle; // OK  Nexter
  double delta_traj_pred = atan(UHcomm) - 0 * 2 * future_curvature * front_sliding_angle;  // Ok simu

  double Ucomm = (wheelbase_ / cos(rear_sliding_angle + DeltaM_ar)) * curvature * std::cos(v1) / v2;

  double Vcomm = (wheelbase_ / cos(rear_sliding_angle + DeltaM_ar)) *
    (v3 * ( (std::cos(v1)) * (std::cos(v1)) * (std::cos(v1)) ) / (v2 * v2) ) +
    std::tan(rear_sliding_angle + DeltaM_ar);

  double delta_ecart = std::atan(Vcomm / (1 + Ucomm * Vcomm + (Ucomm * Ucomm)) ) +
    front_sliding_angle;

  // Calcul partie predictive
  double delta_traj = commandPred_(delta_traj_pred);

  DeltaContr_av = delta_traj;
  DeltaEcartAV = delta_ecart;

  // Calcul commande complete
  return delta_ecart + delta_traj;
}


//------------------------------------------------------------------------------
double FollowTrajectoryPredictiveSliding::commandPred_(double CommFutur)
{
  //  DeltaM_av2=DeltaM_av;
  //  DeltaM_av=front_steering_angle ;

  double feinte, delta, alpha = 0.2;

  Eigen::Matrix3d Fm, Ident, mult2;
  Eigen::Vector3d Gm, Etat;
  Eigen::RowVector3d mult, Cm;
  double CommFut, R1 = 0, R2 = 0, Ai, di;

  //------------------------------------------------
  // Modele interne
  //------------------------------------------------
  Fm(0, 0) = B1_;  Fm(0, 1) = B2_;  Fm(0, 2) = A1_;
  Fm(1, 0) = 1;    Fm(1, 1) = 0;    Fm(1, 2) = 0;
  Fm(2, 0) = 0;    Fm(2, 1) = 0;    Fm(2, 2) = 0;

  Cm(0, 0) = 1;    Cm(0, 1) = 0;   Cm(0, 2) = 0;

  Gm(0, 0) = A0_;
  Gm(1, 0) = 0;
  Gm(2, 0) = 1;
  //------------------------------------------------

  Ident = Eigen::Matrix3d::Identity();
  mult2 = Eigen::Matrix3d::Zero();

  // Etat(0,0) = DeltaM_av - delta_ecart;
  // Etat(1,0) = DeltaM_av2 - DeltaEcartAV;
  // Etat(2,0) = DeltaContr_av;
  // CommFut = CommFutur;
  // feinte      =  DeltaM_av - delta_ecart;


  Etat(0, 0) = DeltaM_av;
  Etat(1, 0) = DeltaM_av2;
  Etat(2, 0) = DeltaContr_av;

  CommFut = CommFutur;
  feinte = DeltaM_av;

  mult(0, 0) = 1;
  mult(0, 1) = 0;
  mult(0, 2) = 0;

  std::vector<double> Ref(horizon_ + 3);
  Ref = reference_(CommFutur, alpha, feinte);

  for (int i = 1; i < (horizon_ + 1); i++) {
    // Calcul intermediaire Ai
    mult2 = Fm;
    Ai = mult * Ident * Gm;
    for (int j = 1; j < (i + 1); j++) {
      Ai = Ai + mult * mult2 * Gm;
      mult2 = mult2 * Fm;
    }

    // Calcul intermediaire di
    CommFut = Ref[i];
    di = CommFut - (Cm * mult2 * Etat);

    R2 = R2 + Ai * di;
    R1 = R1 + (Ai * Ai);
  }
  delta = R2 / R1;
  return delta;
}

//------------------------------------------------------------------------------
std::vector<double> FollowTrajectoryPredictiveSliding::reference_(
  double CommFutur,
  double alpha,
  double feinte1)
{
  std::vector<double> delta(horizon_ + 3);

  delta[1] = feinte1;
  for (int j = 2; j < (horizon_ + 2); j++) {
    delta[j] = CommFutur - (pow(alpha, j - 1)) * (CommFutur - feinte1);
  }

  return delta;
}

//------------------------------------------------------------------------------
double FollowTrajectoryPredictiveSliding::computeRearSteeringAngle_(
  double lateral_deviation,
  double course_deviation,
  double curvature,
  double rear_sliding_angle,
  double desired_lateral_deviation,
  double desired_course_deviation)
{
  if (std::isfinite(KD2_)) {
    double rear_stering_angle_command = -course_deviation - rear_sliding_angle;

    if (std::abs(curvature) <= 0.001) {
      rear_stering_angle_command += std::atan(
        -KD_ * (lateral_deviation - desired_lateral_deviation) / 4 +
        KD2_ * (course_deviation - desired_course_deviation) / KD_);
    } else {
      double alpha = 1 - curvature * (lateral_deviation - desired_lateral_deviation);
      double delta = KD_ * KD_ / alpha - 4 * curvature * KD2_ *
        (course_deviation - desired_course_deviation);
      rear_stering_angle_command += std::atan((KD_ - std::sqrt(delta)) / (2 * curvature));
    }

    if (std::abs(rear_stering_angle_command) > M_PI_4) { //  ???
      rear_stering_angle_command += std::copysign(M_PI_2, -rear_stering_angle_command);
    }

    return rear_stering_angle_command;
  }
  return 0;
}

}  // namespace core
}  // namespace romea
