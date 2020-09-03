//romea
#include "romea_control/observer/SpeedObserverRoland.hpp"

//std
#include <cmath>
#include <iostream>


namespace romea {

const double SpeedObserverRoland::DEFAULT_KD =-2.2;

//-----------------------------------------------------------------------------
SpeedObserverRoland::SpeedObserverRoland(const double & sample_period):
  SpeedObserverRoland(sample_period,DEFAULT_KD)
{

}

//-----------------------------------------------------------------------------
SpeedObserverRoland::SpeedObserverRoland(const double & sample_period,
                                         const double & kd):
  sampling_period_(sample_period),
  kd_(kd),
  longitudinal_deviation_obs_(0),
  leader_linear_speed_obs_(0),
  is_initialized_(false)
{


}

//-----------------------------------------------------------------------------
double SpeedObserverRoland::update(const double & longitudinal_deviation,
                                   const double & course_deviation,
                                   const double & follower_linear_speed,
                                   const double & follower_angular_speed)
{

  if(is_initialized_)
  {
    double error = longitudinal_deviation_obs_ - longitudinal_deviation;

    leader_linear_speed_obs_ = kd_ * error+
        follower_linear_speed * std::cos(course_deviation)+
        0*longitudinal_deviation*follower_angular_speed*std::sin(course_deviation);

    double longitudinal_deviation_obs_dot = leader_linear_speed_obs_ -
        follower_linear_speed * std::cos(course_deviation)  -
        0*longitudinal_deviation*follower_angular_speed*std::sin(course_deviation);

    longitudinal_deviation_obs_ += sampling_period_*longitudinal_deviation_obs_dot;

  }
  else
  {
    longitudinal_deviation_obs_=longitudinal_deviation;
    is_initialized_=true;
  }

  if (leader_linear_speed_obs_>0)
  {
    return leader_linear_speed_obs_;
  }
  else
  {
    return 0.0;
  }
}

//-----------------------------------------------------------------------------
double SpeedObserverRoland::update(const double & longitudinal_deviation,
                                   const double & course_deviation,
                                   const double & follower_linear_speed)
{
  return update(longitudinal_deviation,course_deviation,follower_linear_speed,0.);
}

//-----------------------------------------------------------------------------
void SpeedObserverRoland::reset()
{

  leader_linear_speed_obs_ =0;
  is_initialized_=false;
}

}
