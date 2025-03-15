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

#include "romea_core_control/observer/SlidingObserverBacksteppingSkid.hpp"
#include <romea_core_common/math/EulerAngles.hpp>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

namespace romea::core
{

SlidingObserversBacksteppingSkid::SlidingObserversBacksteppingSkid(
  double step_time, const Parameters & parameters)
: params_(parameters),
  step_time_(step_time),
  beta_r_estime_f_(params_.weight_slip_angle),
  dot_theta_p_estime_f_(params_.weight_linear_speed_disturb),
  dot_epsilon_s_p_estime_f_(params_.weight_angular_speed_disturb),
  w_f_(0),
  v_f_(0),
  epsilon_s_point_f_(0),
  epsilon_theta_point_f_(0)
{
}

void SlidingObserversBacksteppingSkid::update(
  double epsilon_y,
  double epsilon_theta,
  double curvature,
  double speed_longi,
  double w,
  double S_x,
  double S_y,
  double curv_abscissa)
{
  double beta_r_estime = 0;
  double dot_theta_p_estime = 0;
  double dot_epsilon_s_p_estime = 0;
  Eigen::VectorXd desired_speed = Eigen::VectorXd::Zero(3);
  desired_speed(0) = 1;

  // detect if there is a jump in the trajectory (of at least 2 meters) and reset this observer
  if (std::abs(curv_abscissa - epsilon_s_old_) > 2.) {
    reset();
  }

  if (!is_initialized_) {
    reset();
    epsilon_y_estime_ = epsilon_y;
    epsilon_theta_estime_ = epsilon_theta;
    epsilon_theta_old_ = 0;
    epsilon_theta_point_f_.update(
      atan2(sin(epsilon_theta - epsilon_theta_old_), cos(epsilon_theta - epsilon_theta_old_)) /
      step_time_);
    is_initialized_ = true;
    w_f_.update(w);
    v_f_.update(speed_longi);

    S_x_old = S_x;
    S_y_old = S_y;

    epsilon_s_estime_ = curv_abscissa;
    epsilon_s_old_ = epsilon_s_estime_;
    epsilon_s_point_f_.update(0);

    beta_r_estime_ = beta_r_estime_f_.update(beta_r_estime);
    dot_theta_p_estime_ = dot_theta_p_estime_f_.update(dot_theta_p_estime);
    dot_epsilon_s_p_estime_ = dot_epsilon_s_p_estime_f_.update(dot_epsilon_s_p_estime);
  } else {
    // speed_longi=v_f_.update(speed_longi);

    // if (speed_longi > 0)
    //   epsilon_s += sqrt(pow(S_x - S_x_old, 2) + pow(S_y - S_y_old, 2));
    // else
    //   epsilon_s -= sqrt(pow(S_x - S_x_old, 2) + pow(S_y - S_y_old, 2));
    //
    // S_x_old = S_x;
    // S_y_old = S_y;
    epsilon_s = curv_abscissa;

    double epsilon_s_point = epsilon_s_point_f_.update((epsilon_s - epsilon_s_old_) / step_time_);
    double epsilon_theta_point = epsilon_theta_point_f_.update(
      atan2(sin(epsilon_theta - epsilon_theta_old_), cos(epsilon_theta - epsilon_theta_old_)) /
      step_time_);

    epsilon_theta_old_ = epsilon_theta;
    epsilon_s_old_ = epsilon_s;

    double u_theta = w;

    // The N variable defines the number of sub-steps to predict the robot state
    constexpr int N = 1;
    for (int i = 1; i < N + 1; i++) {
      double speed_longi_adapt = speed_longi / cos(beta_r_estime_);

      // double speed_longi_adapt=speed_longi;

      double es = epsilon_s_estime_ - epsilon_s_old_;
      double ey = epsilon_y_estime_ - epsilon_y;
      double e_theta = atan2(
        sin(epsilon_theta_estime_ - epsilon_theta), cos(epsilon_theta_estime_ - epsilon_theta));

      // std::cout << "Skid BS es : " << epsilon_s_estime_ - epsilon_s_old_ << "\n";
      // std::cout << "Skid BS ey : " << epsilon_y_estime_ - epsilon_y << "\n";
      // std::cout << "Skid BS e_theta : "
      //           << atan2(
      //                sin(epsilon_theta_estime_ - epsilon_theta),
      //                cos(epsilon_theta_estime_ - epsilon_theta))
      //           << std::endl;

      //double u_theta=w_f_.update(w);

      double dot_epsilon_y_estime = 0;
      double dot_epsilon_s_estime = 0;
      double dot_epsilon_theta_estime = 0;

      //---------------version en dot_epsilon_s_p---------------------------------

      // //estimation du beta en utilisant la distance
      // //dans ce version il y a une approximation pour le calcul de beta car normalement on ne pet pas se placer en distance
      // //curviligne
      // beta_r_estime = (atan2(params_.ky * ey, (1 - curvature * epsilon_y)) - epsilon_theta);
      //
      // //estimation du glissement longitudinal
      // dot_epsilon_s_p_estime =
      //   params_.ks * es -
      //   speed_longi * cos(epsilon_theta + beta_r_estime) / (1 - curvature * epsilon_y) +
      //   epsilon_s_point;
      //
      // //estimation du glissement en lacet en utilisant le temps
      // dot_theta_p_estime = -u_theta + epsilon_theta_point + params_.k_theta * e_theta +
      //                      curvature * (speed_longi * cos(epsilon_theta + beta_r_estime) /
      //                                     (1 - curvature * epsilon_y) +
      //                                   dot_epsilon_s_p_estime);
      //
      // //on calcul les epsilon y et theta estimé
      // dot_epsilon_y_estime = abs(speed_longi) * sin(epsilon_theta + beta_r_estime);
      // dot_epsilon_s_estime =
      //   speed_longi * cos(epsilon_theta + beta_r_estime) / (1 - curvature * epsilon_y) +
      //   dot_epsilon_s_p_estime;
      // dot_epsilon_theta_estime = w + dot_theta_p_estime -
      //                            curvature * (speed_longi * cos(epsilon_theta + beta_r_estime) /
      //                                           (1 - curvature * epsilon_y) +
      //                                         dot_epsilon_s_p_estime);

      //---------------version en vg----------------------------------------------

      //estimation du beta en utilisant la distance
      beta_r_estime = (atan2(params_.ky * ey, (1 - curvature * epsilon_y)) - epsilon_theta);

      //estimation du glissement longitudinal
      dot_epsilon_s_p_estime = (params_.ks * es + params_.kis * integral + epsilon_s_point) *
                                 (1 - curvature * epsilon_y) / cos(epsilon_theta + beta_r_estime) -
                               speed_longi_adapt;

      // std::cout << "Skid BS dot_epsilon_s_p_estime : " << dot_epsilon_s_p_estime << std::endl;

      //estimation du glissement en lacet en utilisant le temps
      dot_theta_p_estime =
        -u_theta + epsilon_theta_point + params_.k_theta * e_theta +
        curvature * ((speed_longi_adapt + dot_epsilon_s_p_estime) *
                     cos(epsilon_theta + beta_r_estime) / (1 - curvature * epsilon_y));

      //on calcul les epsilon y et theta estimé
      dot_epsilon_s_estime = ((speed_longi_adapt + dot_epsilon_s_p_estime)) *
                             cos(epsilon_theta + beta_r_estime) / (1 - curvature * epsilon_y);

      // std::cout << "Skid BS dot_epsilon_s_estime : " << dot_epsilon_s_estime << std::endl;

      dot_epsilon_y_estime =
        (speed_longi_adapt + dot_epsilon_s_p_estime) * sin(epsilon_theta + beta_r_estime);
      dot_epsilon_theta_estime =
        u_theta + dot_theta_p_estime -
        curvature * ((speed_longi_adapt + dot_epsilon_s_p_estime) *
                     cos(epsilon_theta + beta_r_estime) / (1 - curvature * epsilon_y));

      double dt = step_time_;
      epsilon_s_estime_ +=
        dot_epsilon_s_estime * dt / N + (dot_epsilon_s_estime - dot_epsilon_s_estime_n1_) * dt / 2;
      epsilon_y_estime_ +=
        dot_epsilon_y_estime * dt / N + (dot_epsilon_y_estime - dot_epsilon_y_estime_n1_) * dt / 2;
      epsilon_theta_estime_ += dot_epsilon_theta_estime * dt / N +
                               (dot_epsilon_theta_estime - dot_epsilon_theta_estime_n1_) * dt / 2;
      epsilon_theta_estime_ = betweenMinusPiAndPi(epsilon_theta_estime_);

      beta_r_estime_ = beta_r_estime_f_.update(beta_r_estime);
      dot_theta_p_estime_ = dot_theta_p_estime_f_.update(dot_theta_p_estime);
      dot_epsilon_s_p_estime_ = dot_epsilon_s_p_estime_f_.update(dot_epsilon_s_p_estime);

      dot_epsilon_s_estime_n1_ = dot_epsilon_s_estime;
      dot_epsilon_y_estime_n1_ = dot_epsilon_y_estime;
      dot_epsilon_theta_estime_n1_ = dot_epsilon_theta_estime;

      // std::cout << "dot_epsilon_s_p_estime : " << dot_epsilon_s_p_estime << std::endl;
      // std::cout << "epsilon_s_point : " << epsilon_s_point << std::endl;
      // std::cout << "dot_epsilon_s_estime : " << dot_epsilon_s_estime << std::endl;
      // std::cout << "epsilon_s : " << epsilon_s << std::endl;
      // std::cout << "es : " << es << std::endl;
    }
  }

  ++counter_;
  counter_ = std::min(counter_, 11);

  //cout<<"epsilon_theta : "<<epsilon_theta<<std::endl;
}

void SlidingObserversBacksteppingSkid::reset()
{
  beta_r_estime_ = 0;
  dot_theta_p_estime_ = 0;
  epsilon_theta_estime_ = 0;
  epsilon_theta_old_ = 0;
  epsilon_y_estime_ = 0;
  counter_ = 0;
  is_initialized_ = false;
  epsilon_theta_point_f_.reset();
  w_f_.reset();
  v_f_.reset();
  dot_theta_p_estime_f_.reset();
  beta_r_estime_f_.reset();
  dot_epsilon_s_p_estime_f_.reset();
  epsilon_s_point_f_.reset();
  epsilon_s = 0;
  S_y_old = 0;
  S_x_old = 0;
  integral = 0;
  dot_epsilon_s_estime_n1_ = 0;
  dot_epsilon_y_estime_n1_ = 0;
  dot_epsilon_theta_estime_n1_ = 0;
}

double SlidingObserversBacksteppingSkid::getBetaR() const
{
  return counter_ > 10 ? beta_r_estime_ : 0;
}

double SlidingObserversBacksteppingSkid::getDotThetaP() const
{
  return counter_ > 10 ? dot_theta_p_estime_ : 0;
}

double SlidingObserversBacksteppingSkid::getDotEpsilonSP() const
{
  return counter_ > 10 ? dot_epsilon_s_p_estime_ : 0;
}

}  // namespace romea::core
