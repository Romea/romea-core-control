#ifndef _romea_FollowTrajectoryClassicSliding_hpp_
#define _romea_FollowTrajectoryClassicSliding_hpp_

//std
#include <utility>
#include <vector>

//romea
#include "../FrontRearData.hpp"

namespace romea {


class FollowTrajectoryClassicSliding
{

public :

  struct Parameters
  {
    double front_kp;
    double rear_kp;
  };

public:

  FollowTrajectoryClassicSliding(const double & wheel_base,
                                 const Parameters & parameters);

  FrontRearData computeSteeringAngles(const double&  lateral_deviation,
                                      const double & course_deviation,
                                      const double & curvature,
                                      const double & front_sliding_angle,
                                      const double & rear_sliding_angle,
                                      const double & rear_steering_angle,
                                      const double & front_maximal_steering_angle,
                                      const double & rear_maximal_steering_angle,
                                      const double & desired_lateral_deviation,
                                      const double & desired_course_deviation);


  void setFrontKP(const double & kp);

private :


  double computeFrontSteeringAngle_(const double & lateral_deviation,
                                    const double & course_deviation,
                                    const double & curvature,
                                    const double & front_sliding_angle,
                                    const double & rear_sliding_angle,
                                    const double & desired_lateral_deviation);


  double computeRearSteeringAngle_(const double& lateral_deviation,
                                   const double& course_deviation,
                                   const double& curvature,
                                   const double &rear_sliding_angle);


private :

  double wheelbase_;
  double KD_;
  double KP_;
  double KD2_;
  double DeltaM_ar;
};

}

#endif
