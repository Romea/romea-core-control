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

#include "romea_core_control/observer/SlidingObserverPicardSkidLyapunov.hpp"

#include <algorithm>
#include <cmath>
#include <romea_core_common/math/EulerAngles.hpp>

namespace romea::core
{

SlidingObserversPicardSkidLyapunov::SlidingObserversPicardSkidLyapunov(
  double step_time, const Parameters & parameters)
: params_(parameters),
  step_time_(step_time)
{
}

void SlidingObserversPicardSkidLyapunov::update(
  double epsilon_x,
  double epsilon_y,
  double epsilon_theta,
  double longi_speed,
  double angular_speed)
{
  X_R = epsilon_x;
  Y_R = epsilon_y;
  Theta_R = epsilon_theta;
  if (!is_initialized_) {
    epsilon_x_estime_ = epsilon_x;
    epsilon_y_estime_ = epsilon_y;
    epsilon_theta_estime_ = epsilon_theta;
    is_initialized_ = true;
  } else {
    double u_theta = angular_speed;

    int N = 1;
    for (int i = 1; i < N + 1; i++) {
      double longi_speed_adapt = longi_speed / cos(beta_r_estime_);
      // double longi_speed_adapt = longi_speed;

      //calcul des erreurs
      double ex = (epsilon_x_estime_ - epsilon_x);
      double ey = (epsilon_y_estime_ - epsilon_y);
      double e_theta = atan2(
        sin(epsilon_theta_estime_ - epsilon_theta), cos(epsilon_theta_estime_ - epsilon_theta));

      /*cout<<"Skid L epsilon_x : "<<epsilon_x<<endl;
            cout<<"Skid L epsilon_y : "<<epsilon_y<<endl;
            cout<<"Skid L epsilon_theta : "<<epsilon_theta<<endl;

            cout<<"Skid L ex : "<<ex<<endl;
            cout<<"Skid L ey : "<<ey<<endl;
            cout<<"Skid L e_theta : "<<e_theta<<endl;*/

      //calcul de l'état estimé
      double dot_epsilon_x_estime =
        (longi_speed_adapt + dot_epsilon_s_p_estime_) * cos(epsilon_theta + beta_r_estime_) +
        params_.kex * ex;
      double dot_epsilon_y_estime =
        (longi_speed_adapt + dot_epsilon_s_p_estime_) * sin(epsilon_theta + beta_r_estime_) +
        params_.key * ey;
      double dot_epsilon_theta_estime = u_theta + dot_theta_p_estime_ + params_.ketheta * e_theta;

      /*cout<<"Skid L dot_epsilon_x_estime : "<<dot_epsilon_x_estime<<endl;
            cout<<"Skid L dot_epsilon_y_estime : "<<dot_epsilon_y_estime<<endl;
            cout<<"Skid L dot_epsilon_theta_estime : "<<dot_epsilon_theta_estime<<endl;*/

      double h11 = 0.5 * -sin(epsilon_theta + beta_r_estime_) * ey;
      double h12 = 0.5 * (-sin(epsilon_theta + beta_r_estime_) * ex -
                          (longi_speed_adapt + dot_epsilon_s_p_estime_) *
                            cos(epsilon_theta + beta_r_estime_) * ey);
      double h21 = 0.5 * cos(epsilon_theta + beta_r_estime_) * ey;
      double h22 = 0.5 * (cos(epsilon_theta + beta_r_estime_) * ex -
                          (longi_speed_adapt + dot_epsilon_s_p_estime_) *
                            sin(epsilon_theta + beta_r_estime_) * ey);

      /*cout<<"Skid L h11 : "<<h11<<endl;
            cout<<"Skid L h12 : "<<h12<<endl;
            cout<<"Skid L h21 : "<<h21<<endl;
            cout<<"Skid L h22 : "<<h22<<endl;*/

      double dot_beta =
        params_.kbeta *
        (-(longi_speed_adapt + dot_epsilon_s_p_estime_) * sin(epsilon_theta + beta_r_estime_) * ex +
         (longi_speed_adapt + dot_epsilon_s_p_estime_) * cos(epsilon_theta + beta_r_estime_) * ey +
         h12 * ex * 0 + h22 * ey * 0);

      double dot_dot_vg =
        params_.kdotsp * (cos(epsilon_theta + beta_r_estime_) * ex +
                          sin(epsilon_theta + beta_r_estime_) * ey + h11 * 0 * ex + h21 * 0 * ey);

      double dot_dot_epsilon_theta_p = params_.kdotthetap * e_theta;


      // dot_beta=0.0;
      dot_dot_vg =0.0;
      // dot_dot_epsilon_theta_p = 0.0;

      //integration de l'etat estimé

      double dt = step_time_;
      // cout  <<"Step_Time_ : "<< step_time_ << std::endl;
      epsilon_x_estime_ += dot_epsilon_x_estime * dt / N +
                           0*((dot_epsilon_x_estime - dot_epsilon_x_estime_n1_) / dt) * (dt * dt / 2);
      epsilon_y_estime_ += dot_epsilon_y_estime * dt / N +
                           0*((dot_epsilon_y_estime - dot_epsilon_y_estime_n1_) / dt) * (dt * dt / 2);
      epsilon_theta_estime_ +=
        dot_epsilon_theta_estime * dt / N +
        0*((dot_epsilon_theta_estime - dot_epsilon_theta_estime_n1_) / dt) * (dt * dt / 2);
      epsilon_theta_estime_ = atan2(sin(epsilon_theta_estime_), cos(epsilon_theta_estime_));

      /*cout<<"Skid L epsilon_x_estime_ : "<<epsilon_x_estime_<<endl;
            cout<<"Skid L epsilon_y_estime_ : "<<epsilon_y_estime_<<endl;
            cout<<"Skid L epsilon_theta_estime_ : "<<epsilon_theta_estime_<<endl;*/

      dot_theta_p_estime_ +=
        dot_dot_epsilon_theta_p * dt / N + 0*((dot_beta - dot_beta_n1_) / dt) * (dt * dt / 2);
      beta_r_estime_ += dot_beta * dt / N + 0*((dot_dot_vg - dot_dot_vg_n1_) / dt) * (dt * dt / 2);
      dot_epsilon_s_p_estime_ +=
        dot_dot_vg * dt / N +
        0*((dot_dot_epsilon_theta_p - dot_dot_epsilon_theta_p_n1_) / dt) * (dt * dt / 2);

      dot_epsilon_x_estime_n1_ = dot_epsilon_x_estime;
      dot_epsilon_y_estime_n1_ = dot_epsilon_y_estime;
      dot_epsilon_theta_estime_n1_ = dot_epsilon_theta_estime;
      dot_beta_n1_ = dot_beta;
      dot_dot_vg_n1_ = dot_dot_vg;
      dot_dot_epsilon_theta_p_n1_ = dot_dot_epsilon_theta_p;
    }
  }

  counter_ = std::min(counter_ + 1, end_counter_);
}

void SlidingObserversPicardSkidLyapunov::reset()
{
  beta_r_estime_ = 0;
  dot_theta_p_estime_ = 0;
  dot_epsilon_s_p_estime_ = 0;
  counter_ = 0;
  epsilon_x_estime_ = 0;
  epsilon_y_estime_ = 0;
  epsilon_theta_estime_ = 0;

  dot_epsilon_x_estime_n1_ = 0;
  dot_epsilon_y_estime_n1_ = 0;
  dot_epsilon_theta_estime_n1_ = 0;
  dot_beta_n1_ = 0;
  dot_dot_vg_n1_ = 0;
  dot_dot_epsilon_theta_p_n1_ = 0;

  is_initialized_ = false;
}

double SlidingObserversPicardSkidLyapunov::getBetaR() const
{
  return counter_ >= end_counter_ ? beta_r_estime_ : 0;
}

double SlidingObserversPicardSkidLyapunov::getDotThetaP() const
{
  return counter_ >= end_counter_ ? dot_theta_p_estime_ : 0;
}

double SlidingObserversPicardSkidLyapunov::getDotEpsilonSP() const
{
  return counter_ >= end_counter_ ? dot_epsilon_s_p_estime_ : 0;
}

double SlidingObserversPicardSkidLyapunov::getXEstime() const
{
  return counter_ >= end_counter_ ? epsilon_x_estime_ : 0;
}
double SlidingObserversPicardSkidLyapunov::getYEstime() const
{
  return counter_ >= end_counter_ ? epsilon_y_estime_ : 0;
}
double SlidingObserversPicardSkidLyapunov::getThetaEstime() const
{
  return counter_ >= end_counter_ ? epsilon_theta_estime_ : 0;
}

double SlidingObserversPicardSkidLyapunov::getXR() const
{
  return counter_ >= end_counter_ ? X_R : 0;
}

double SlidingObserversPicardSkidLyapunov::getYR() const
{
  return counter_ >= end_counter_ ? Y_R : 0;
}

double SlidingObserversPicardSkidLyapunov::getThetaR() const
{
  return counter_ >= end_counter_ ? Theta_R : 0;
}

}  // namespace romea::core
