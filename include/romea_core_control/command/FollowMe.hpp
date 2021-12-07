#ifndef _romea_CommandFollowMe_hpp
#define _romea_CommandFollowMe_hpp

//std
#include <utility>

//romea
#include "../FrontRearData.hpp"

namespace romea {

class FollowMe
{
public :

  struct Parameters
  {
    double kp;
    double kd;
    double ki;
    double kdd;
  };

public:

  FollowMe(const double & sampling_period,
           const Parameters & parameters);

  void setDesiredLateralDeviation(const double& desired_lat_dev);

  const double & getDesiredLateralDeviation()const;

  double computeAngularSpeed(const double & lateral_deviation,
                             const double & course_deviation,
                             const double & maximal_angular_speed,
                             double & omega_d,
                             double & theta_error);

  FrontRearData computeSteeringAngles(const double &wheelbase,
                                      const double & lateral_deviation,
                                      const double & course_deviation,
                                      const double & rear_steering_angle,
                                      const double &maximal_front_steering_angle,
                                      const double &maximal_rear_steering_angle,
                                      double & omega_d,
                                      double & theta_consigne,
                                      const double & vitesse,
                                      const double & yaw_rate_leader,
                                      const double & desired_longitudinal_distance);

  FrontRearData computeSteeringAngles(const double &wheelbase,
                                      const double &lateral_deviation,
                                      const double &course_deviation,
                                      const double &courbure,
                                      const double &speed,
                                      const double &rear_steering_angle,
                                      const double &rear_sliding_angle,
                                      const double &front_sliding_angle,
                                      const double &minimal_theta,
                                      const double &maximal_theta,
                                      const double &maximal_front_steering_angle,
                                      const double &maximal_rear_steering_angle,
                                      double &omega_d,
                                      double &theta_consigne);


  FrontRearData computeSteeringAngles(const double &wheelbase,
                                      const double &lateral_deviation,
                                      const double &course_deviation,
                                      const double &curvature,
                                      const double & speed,
                                      const double &rear_steering_angle,
                                      const double &rear_sliding_angle,
                                      const double &front_sliding_angle,
                                      const double &minimal_theta,
                                      const double &maximal_theta,
                                      const double &maximal_front_steering_angle,
                                      const double &maximal_rear_steering_angle,
                                      const double &courbe0,
                                      const double &courbe1,
                                      const double &courbe2,
                                      const double &lambda,
                                      double &omega_d,
                                      double &theta_consigne);


  void reset();

private:

  double sampling_period_;

  double kp_;
  double ki_;
  double kd_;
  double kdd_;

  double desired_lat_dev_;
  double integrated_omega_;

};

}

#endif
