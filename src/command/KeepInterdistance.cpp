//romea
#include "romea_control/command/KeepInterdistance.hpp"
#include <romea_common/signal/FirstOrderButterworth.hpp>

//std
#include <cmath>
#include <iostream>

namespace {
const double KI =0.2;
}

namespace romea {

//-----------------------------------------------------------------------------
KeepInterdistance::KeepInterdistance(double sampling_period):
  sampling_period_(sampling_period),
  KI_(KI),
  interdistance_error_(0),
  integrated_longitudinal_deviation_(0)
{

}

//-----------------------------------------------------------------------------
void KeepInterdistance::reset()
{
  integrated_longitudinal_deviation_=0;
}

//-----------------------------------------------------------------------------
double KeepInterdistance::computeFollowerSpeed(const double & desired_interdistance,
                                               const double & interdistance,
                                               const double & follower_lateral_deviation,
                                               const double & follower_linear_speed,
                                               const double & follower_maximal_linear_speed)
{
  static romea::FirstOrderButterworth f(0.25);

  updateLongitudinalDeviation_(interdistance,desired_interdistance,follower_linear_speed);

  double follower_linear_speed_command = 0;
  if(std::abs(interdistance_error_) > 0.1)
  {
    //double linear_speed_gain = 0.8;
    double linear_speed_gain = 0.3 + 0.5/(1+4*std::fabs(follower_lateral_deviation));

    std::cout << integrated_longitudinal_deviation_ << " "<< linear_speed_gain <<" "<<interdistance_error_<<" "<< KI_<<" "<<linear_speed_gain*interdistance_error_ +integrated_longitudinal_deviation_ * KI_<<std::endl;

    follower_linear_speed_command = f.update(linear_speed_gain*interdistance_error_ +
                                             integrated_longitudinal_deviation_ * KI_);
  }
  else
  {
    f.update(0);
  }

  return clamp(follower_linear_speed_command,0.,follower_maximal_linear_speed);
}


//-----------------------------------------------------------------------------
double KeepInterdistance::computeFollowerSpeed(const double & desired_interdistance,
                                               const double & interdistance,
                                               const double & leader_linear_speed,
                                               const double & follower_maximal_linear_speed)
{
  static romea::FirstOrderButterworth f(0.25);

  double offset_free_leader_linear_speed = offsetFreeLinearSpeed_(leader_linear_speed);

  double interdistance_error= interdistance-desired_interdistance;

  double follower_linear_speed_command = 0;
  if(std::abs(interdistance_error) > 0.1)
  {
    double linear_speed_gain = 0.5;
    follower_linear_speed_command = f.update(offset_free_leader_linear_speed+linear_speed_gain *interdistance_error);
  }
  else
  {
    f.update(0);
  }

  return clamp(follower_linear_speed_command,0.,follower_maximal_linear_speed);

}

//-----------------------------------------------------------------------------
double KeepInterdistance::computeFollowerSpeed(const double & desired_interdistance,
                                               const double & desired_lateral_deviation,
                                               const double & follower_interdistance,
                                               const double & leader_interdistance,
                                               const double & leader_linear_speed,
                                               const double & follower_lateral_deviation,
                                               const double & follower_course_deviation,
                                               const double & follower_linear_speed,
                                               const double & follower_maximal_linear_speed,
                                               const double & /*desired_angular_deviation*/,
                                               const double & yaw_rate_leader)
{

  static romea::FirstOrderButterworth f(0.25);


  double offset_free_leader_linear_speed = offsetFreeLinearSpeed_(leader_linear_speed);
  updateLongitudinalDeviation_(follower_interdistance,desired_interdistance,follower_linear_speed);

  std::cout << " desired_interdistance " << desired_interdistance<<std::endl;
  std::cout << " desired_lateral_deviation " << desired_lateral_deviation<<std::endl;
  std::cout << " follower_interdistance " << follower_interdistance <<std::endl;
  std::cout << " leader_interdistance " << leader_interdistance<<std::endl;
  std::cout << " leader_linear_speed " << offset_free_leader_linear_speed<<std::endl;
  std::cout << " offset_free_leader_linear_speed " << offset_free_leader_linear_speed<<std::endl;
  std::cout << " follower_lateral_deviation " << follower_lateral_deviation << std::endl;
  std::cout << " follower_course_deviation " << follower_course_deviation << std::endl;
  std::cout << " follower_linear_speed " << follower_linear_speed << std::endl;

  double follower_linear_speed_command=0;
  //if(std::abs(interdistance_error_)) {

  double t_pred = 0.8;
  //  double linear_speed_gain = 0.3/(1+0*4*std::abs(follower_course_deviation-desired_angular_deviation)) + 0.3/(1+4*std::abs(0*follower_lateral_deviation));
  //  double linear_speed_gain = .5;
  double linear_speed_gain = 1.0;
  double k_vitesse_leader = 1;

  double clamped_follower_course_deviation= follower_course_deviation;
  if(fabs(clamped_follower_course_deviation) > (70*M_PI/180))
  {
    clamped_follower_course_deviation = copysign((70*M_PI/180), follower_course_deviation);
  }

  if (std::abs(desired_interdistance)>1.5)
  {

    follower_linear_speed_command = 1*(leader_linear_speed*std::cos(0*clamped_follower_course_deviation)-0*yaw_rate_leader*follower_lateral_deviation)/std::cos(1*clamped_follower_course_deviation)
        + linear_speed_gain/std::cos(0*clamped_follower_course_deviation) * (0.8*(follower_interdistance)+0.2*(leader_interdistance) - desired_interdistance + 0*offset_free_leader_linear_speed*std::cos(clamped_follower_course_deviation)*t_pred)
        + 0*integrated_longitudinal_deviation_ * KI_;
  }
  else
  {
    //    if(std::fabs(interdistance) > 0.6) k_vitesse_leader = 0.5;
    //    else k_vitesse_leader = 1.0;
    //    if(offset_free_leader_linear_speed < 0.1) integrated_longitudinal_deviation_ = 0;

    follower_linear_speed_command = k_vitesse_leader*(leader_linear_speed*std::cos(0*clamped_follower_course_deviation)+yaw_rate_leader*follower_lateral_deviation)/std::cos(1*clamped_follower_course_deviation)
        + linear_speed_gain/std::cos(1*clamped_follower_course_deviation) * (0.2*(follower_interdistance)+0.8*(leader_interdistance) - desired_interdistance + 0*offset_free_leader_linear_speed*std::cos(clamped_follower_course_deviation)*t_pred)
        + 0*integrated_longitudinal_deviation_ * KI_*1.0;

    //    if (fabs(0.2*(follower_interdistance)+0.8*(leader_interdistance) - desired_interdistance)>1){
    //         follower_linear_speed_command=std::min(follower_linear_speed_command,0.5);
    //    }

  }


  //  if (std::abs(follower_course_deviation)>85*M_PI/180)
  //  {
  //    follower_linear_speed_command=0;
  //  }

  //  if (std::abs(desired_interdistance)>1)
  //  {
  //    if (std::sqrt(interdistance*interdistance+follower_lateral_deviation*follower_lateral_deviation)<.8)
  //    {
  //      follower_linear_speed_command=0;
  //    }
  //  }
  //  else
  //  {
  //    if (std::sqrt(interdistance*interdistance+(follower_lateral_deviation+desired_lateral_deviation)*(follower_lateral_deviation+desired_lateral_deviation))<.8)
  //    {
  //      follower_linear_speed_command=0;
  //    }
  //  }

  //  if (ObstacleDetect==1 && follower_speed>0.5)
  //    follower_speed=0.5;

  follower_linear_speed_command = f.update(follower_linear_speed_command);
  follower_linear_speed_command= clamp(follower_linear_speed_command,0.,follower_maximal_linear_speed);
  std::cout <<  " follower_maximal_linear_speed " << follower_maximal_linear_speed<<std::endl;
  std::cout <<  " follower_linear_speed_command " << follower_linear_speed_command<<std::endl;
  return follower_linear_speed_command;
}

//-----------------------------------------------------------------------------
double KeepInterdistance::computeFollowerSpeed(const double & desired_interdistance,
                                               const double & interdistance,
                                               const double & leader_lateral_deviation,
                                               const double & leader_course_deviation,
                                               const double & leader_curvature,
                                               const double & leader_linear_speed,
                                               const double & /*leader_rear_streering_angle*/,
                                               const double & leader_rear_sliding_angle,
                                               const double & follower_lateral_deviation,
                                               const double & follower_course_deviation,
                                               const double & follower_curvature,
                                               const double & /*follower_linear_speed*/,
                                               const double & follower_rear_streering_angle,
                                               const double & follower_rear_sliding_angle,
                                               const double & follower_maximal_linear_speed)
{

  double offset_free_leader_linear_speed = offsetFreeLinearSpeed_(leader_linear_speed);

  double follower_linear_speed_command =0;

  if(interdistance> 5)
  {
    double tmp = 1 - follower_lateral_deviation * follower_curvature;
    double K=0.2;
    //if (( Interdistance - desired_interdistance_ )>2) K=0.7;

    follower_linear_speed_command = tmp / std::cos( follower_course_deviation + follower_rear_sliding_angle + follower_rear_streering_angle )
        * ( offset_free_leader_linear_speed * std::cos( leader_course_deviation + leader_rear_sliding_angle )
            / ( 1 - leader_lateral_deviation * leader_curvature )
            + K * ( interdistance - desired_interdistance ) );

  }


  return clamp(follower_linear_speed_command,0.,follower_maximal_linear_speed);
}


//-----------------------------------------------------------------------------
double KeepInterdistance::offsetFreeLinearSpeed_(const double & linear_speed)
{
  if(std::abs(linear_speed) > 0.1)
  {
    double offset = copysign(0.1, linear_speed);
    return linear_speed - offset;
  }
  else
  {
    return 0;
  }
}

//-----------------------------------------------------------------------------
void KeepInterdistance::updateLongitudinalDeviation_(const double & interdistance,
                                                     const double & desired_interdistance,
                                                     const double & /*follower_linear_speed*/)
{

  interdistance_error_= interdistance-desired_interdistance;

  integrated_longitudinal_deviation_ += sampling_period_*(interdistance_error_);

  if (std::abs(integrated_longitudinal_deviation_) > 1)
  {
    integrated_longitudinal_deviation_ = std::copysign(1.5, integrated_longitudinal_deviation_);
  }
  if ((interdistance_error_ < -0.3) && (interdistance_error_ > 0.3))
    integrated_longitudinal_deviation_=0;

}

}
