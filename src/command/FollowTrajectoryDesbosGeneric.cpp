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

// romea core
#include <romea_core_common/math/Algorithm.hpp>

// local
#include "romea_core_control/command/FollowTrajectoryDesbosGeneric.hpp"

namespace romea::core
{

FollowTrajectoryDesbosGeneric::FollowTrajectoryDesbosGeneric(const Parameters & parameters)
: kp_(parameters.kp),
  kd_(parameters.kd),
  ks_(parameters.ks),
  adaptive_gains_(parameters.adaptive_gains)
{
}

void FollowTrajectoryDesbosGeneric::reset()
{
  integrated_omega_ = 0;
}

void FollowTrajectoryDesbosGeneric::set_desired_lateral_deviation(double desired_lat_dev)
{
  if (std::abs(desired_lat_dev - desired_lat_dev_) > 0.1) {
    desired_lat_dev_ = desired_lat_dev;
    integrated_omega_ = 0;
  }
}

void FollowTrajectoryDesbosGeneric::set_gains(double kp, double kd, double ks)
{
  kp_ = kp;
  kd_ = kd;
  ks_ = ks;
}

double FollowTrajectoryDesbosGeneric::compute_angular_speed(
  double lateral_deviation,
  double course_deviation,
  double /*maximal_angular_speed*/,
  double courbure,
  double future_courbure,
  double speed,
  double /*longitudinal_speed_command*/,
  // generic slip
  double /*lateral_slip*/,
  double /*angular_slip*/,
  // skid slip
  double /*speed_slip*/,
  double beta,
  double /*angular_skid_slip*/,
  double tau,
  double & /*omega_d*/,
  double & /*theta_error*/,
  double & /*osc_eta*/,
  double & /*osc_amp*/)
{
  // double error;

  // if(stop_lcp == 1){
  //   kp_ = 0;
  //   kd_ = 0;
  // }
  // else
  // {
  //   kd_ = -1;
  // }
  if (adaptive_gains_) {
    // if( (courbure < 0.05) && (courbure > -0.05))
    // {
    //   if(osc_eta > 0.0){
    //     kd_ = kd_ + 4*osc_amp*0.1;
    //   }
    //   else
    //   {
    //     kd_ = kd_ - 0.02*0.1;
    //   }
    // }
    // else
    // {
    //   kd_ = (double) kd_ + 1./20.*(- 1./(tau) - kd_);
    // }
    // k=3/Trep
    // kd_ = 3/(2*2*tau); //Trep_psid=2*Trep_psi  ;;; Trep_psi=2*tau
    // kp_ = 3/(2*speed*2*2*tau); //Trep_y=2*Trep_psid

    kd_ = kd_ + 1. / 5. * (-1. / (1.0 * tau) - kd_);
    kd_ = std::min(-0.02, std::max(-5., kd_));

    double zeta = 1;
    kp_ = kd_ * kd_ / (4 * zeta * zeta);
  }

  // double N;
  // N = 3; //std::min( std::max(2*std::abs(lateral_deviation)+3, 3.),4. );
  // kp_ = kd_ / (N*longitudinal_speed_command); // HARD CODED COMMANDED SPEED (0.5)
  // kp_ = std::min(-0.02,std::max(-4.,kp_));

  // double activ_obs = 0;
  // if (fabs(future_courbure) > 0.1) {
  //   activ_obs = 0;
  // }

  // omega_d = std::atan2(kp_*(lateral_deviation-desired_lat_dev_)-lateral_slip*activ_obs,1-courbure*lateral_deviation) - beta;
  // // omega_d = std::atan2(kp_*copysign(sqrt(abs(lateral_deviation-desired_lat_dev_)),lateral_deviation-desired_lat_dev_)-lateral_slip*activ_obs,1-courbure*lateral_deviation) - beta;

  // if(std::abs(omega_d) > SKID_STEERING_MAXIMAL_OMEGA_D)
  // {
  //   omega_d = copysign(SKID_STEERING_MAXIMAL_OMEGA_D, omega_d);
  // }

  // omega_d = 0;
  // BACKSTEPPING CLASSIQUE --- kp et kd negatifs kd=-1.6  kp=-1.6/3
  // kp_ = kd_ / (3*longitudinal_speed_command);
  // omega_d = std::atan2(kp_*(lateral_deviation-desired_lat_dev_)-lateral_slip*activ_obs,1-courbure*lateral_deviation) - beta;
  // error = atan2(sin(course_deviation - omega_d),cos(course_deviation - omega_d));
  // double angular_speed_command=kd_*(error)+ (speed+speed_slip)*future_courbure*cos(course_deviation+beta)/(1-courbure*lateral_deviation) - angular_slip*activ_obs - angular_skid_slip;

  // CHAINED CONTROLLER - LINEARISATION EXACTE --- kp et kd positifs KD=2
  double angular_speed_command =
    speed * ((pow(cos(course_deviation), 3)) / pow(1 - courbure * lateral_deviation, 2) *
               (-kp_ * lateral_deviation -
                kd_ * (1 - courbure * lateral_deviation) * tan(course_deviation)) +
             future_courbure * cos(course_deviation + beta) / (1 - courbure * lateral_deviation));

  // SMC CONTROLLER --- kp et lamda positifs sign ou arctan lambda=kd=0.5 kp=0.2
  // double lambda = kd_;
  // double s = lambda * lateral_deviation + course_deviation;
  // double angular_speed_command = -kp_*atan(s) - lambda*speed*sin(course_deviation) + future_courbure*cos(course_deviation+beta)/(1-courbure*lateral_deviation);
  // double angular_speed_command = -kp_*atan(s) - lambda*speed*sin(course_deviation) + future_courbure*cos(course_deviation+beta)/(1-courbure*lateral_deviation);

  // AUTRE (!) (!)
  // error = tan(course_deviation) - tan(omega_d);
  // double angular_speed_command = kd_*(error)*pow(cos(course_deviation),2) + (speed+speed_slip)*future_courbure*cos(course_deviation+beta)/(1-courbure*lateral_deviation) - angular_slip*activ_obs - angular_skid_slip;
  // A TESTER UN JOUR (ajout de cos2(tke) dans la commande) + erreur avec des tan

  std::cout << "----------------------------------\n";
  std::cout << "kp: " << kp_;
  std::cout << ", kd: " << kd_;
  std::cout << ", tau: " << tau << "\n";
  std::cout << "ang_speed: " << angular_speed_command << std::endl;
  // std::cout << "activ_obs: " << activ_obs << std::endl;

  return angular_speed_command;
}

//-----------------------------------------------------------------------------
GenericCommandsData FollowTrajectoryDesbosGeneric::compute_commands(
  double lateral_deviation,
  double course_deviation,
  double longitudinal_deviation,
  double desired_speed,
  double /*maximal_angular_speed*/,
  double courbure,
  double future_courbure,
  double speed,
  double longitudinal_speed_command,
  // generic slip
  double lateral_slip,
  double angular_slip,
  double longitudinal_slip,
  // skid slip
  double speed_slip,
  double beta,
  double angular_skid_slip,
  double tau,
  double & omega_d,
  double & theta_error,
  double & osc_eta,
  double & osc_amp)
{
  double angular_speed_command = compute_angular_speed(
    lateral_deviation,
    course_deviation,
    std::numeric_limits<double>::max(),
    courbure,
    future_courbure,
    speed,
    longitudinal_speed_command,
    lateral_slip,
    angular_slip,
    speed_slip,
    beta,
    angular_skid_slip,
    omega_d,
    theta_error,
    tau,
    osc_eta,
    osc_amp);

  double longitudinal_speed_setpoint = compute_velocity(
    lateral_deviation,
    course_deviation,
    longitudinal_deviation,
    desired_speed,
    courbure,
    longitudinal_slip,
    speed_slip,
    beta);

  return {longitudinal_speed_setpoint, angular_speed_command};
}

//-----------------------------------------------------------------------------
double FollowTrajectoryDesbosGeneric::compute_velocity(
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

}  // namespace romea::core
