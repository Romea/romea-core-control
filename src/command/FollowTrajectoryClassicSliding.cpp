//romea
#include "romea_core_control/command/FollowTrajectoryClassicSliding.hpp"
#include <romea_core_common/math/Algorithm.hpp>


//Eigen
#include <Eigen/Dense>
#include <iostream>

//namespace {
//const double DEFAULT_KD = 0.7;
//const double DEFAULT_KP =DEFAULT_KD*DEFAULT_KD/4;
//const double DEFAULT_KD2 = 0.5;
//}

namespace romea {


//-----------------------------------------------------------------------------
FollowTrajectoryClassicSliding::FollowTrajectoryClassicSliding(const double & whee_base,
                                                               const Parameters & parameters):
  wheelbase_(whee_base),
  KD_(parameters.front_kp),
  KP_(KD_*KD_/4),
  KD2_(parameters.rear_kp),
  DeltaM_ar(0)
{
}

//-----------------------------------------------------------------------------
void FollowTrajectoryClassicSliding::setFrontKP(const double & kp)
{
  KD_=kp;
  KP_=(KD_*KD_/4.);
}
//-----------------------------------------------------------------------------
FrontRearData FollowTrajectoryClassicSliding::computeSteeringAngles(const double&  lateral_deviation,
                                                                    const double & course_deviation,
                                                                    const double & curvature,
                                                                    const double & front_sliding_angle,
                                                                    const double & rear_sliding_angle,
                                                                    const double & rear_steering_angle,
                                                                    const double & front_maximal_steering_angle,
                                                                    const double & rear_maximal_steering_angle,
                                                                    const double & desired_lateral_deviation,
                                                                    const double & /*desired_course_deviation*/)
{
  //compute front steering angle
  double front_steering_angle_ = computeFrontSteeringAngle_(lateral_deviation,
                                                            course_deviation,
                                                            curvature,
                                                            -front_sliding_angle,
                                                            rear_sliding_angle+rear_steering_angle,
                                                            desired_lateral_deviation);

  front_steering_angle_ = clamp(front_steering_angle_,
                                -front_maximal_steering_angle,
                                front_maximal_steering_angle);

  //compute rear steering angle
  double rear_steering_angle_ = computeRearSteeringAngle_(lateral_deviation-desired_lateral_deviation,
                                                          course_deviation,
                                                          curvature,
                                                          rear_sliding_angle);


  rear_steering_angle_ = clamp(rear_steering_angle_,
                               -rear_maximal_steering_angle,
                               rear_maximal_steering_angle);

  return {front_steering_angle_,rear_steering_angle_};
}

//-----------------------------------------------------------------------------
double FollowTrajectoryClassicSliding::computeFrontSteeringAngle_(const double&  lateral_deviation,
                                                                  const double & course_deviation,
                                                                  const double & curvature,
                                                                  const double & front_sliding_angle,
                                                                  const double & rear_sliding_angle,
                                                                  const double & desired_lateral_deviation)
{

  double a1 = course_deviation+rear_sliding_angle+DeltaM_ar;
  double a2 = 1-curvature*lateral_deviation;
  double a3 = -KD_*a2*std::tan(a1)-KP_*(lateral_deviation-desired_lateral_deviation)+
      curvature*a2*((std::tan(a1))*(std::tan(a1)));

  return std::atan((wheelbase_/std::cos(rear_sliding_angle+DeltaM_ar)) *
                   (curvature*std::cos(a1)/a2 + a3*( (std::cos(a1))*(std::cos(a1))*(std::cos(a1)))/(a2*a2)) +
                   std::tan(rear_sliding_angle)) + front_sliding_angle;

}

//------------------------------------------------------------------------------
double FollowTrajectoryClassicSliding::computeRearSteeringAngle_(const double& lateral_deviation,
                                                                 const double& course_deviation,
                                                                 const double& curvature,
                                                                 const double& rear_sliding_angle)
{
  double rear_stering_angle_ = -course_deviation -rear_sliding_angle;

  if(std::abs(curvature) <= 0.001)
  {
    rear_stering_angle_ += std::atan(-KD_*lateral_deviation/4 + KD2_*(course_deviation-5/180.*M_PI*0)/KD_);
  }
  else
  {
    double alpha = 1-curvature*lateral_deviation;
    double delta = KD_*KD_/alpha - 4*curvature*KD2_*(course_deviation-5/180.*M_PI*0);
    rear_stering_angle_ += std::atan((KD_-std::sqrt(delta)) / (2*curvature));
  }

  if(std::abs(rear_stering_angle_) > M_PI_4) //???
  {
    rear_stering_angle_ += std::copysign(M_PI_2,-rear_stering_angle_);
  }

  return rear_stering_angle_;
}

}
