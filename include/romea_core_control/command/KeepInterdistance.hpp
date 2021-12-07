#ifndef _romea_CommandKeepInterdistance_hpp_
#define _romea_CommandKeepInterdistance_hpp_

//std
#include <vector>

//romea
#include <romea_core_common/signal/FirstOrderButterworth.hpp>
#include <romea_core_common/math/Algorithm.hpp>

namespace romea {


class KeepInterdistance
{
public:

  KeepInterdistance(double sampling_period);

  double computeFollowerSpeed(const double & desired_interdistance,
                              const double & interdistance,
                              const double & leader_linear_speed,
                              const double & follower_maximal_linear_speed);


  double computeFollowerSpeed(const double & desired_interdistance,
                              const double & interdistance,
                              const double & leader_linear_speed,
                              const double & follower_maximal_linear_speed,
                              const double & follower_lat_dev,
                              const double & follower_ang_dev,
                              const double & courbure,
                              const double & follower_linear_speed);


  double computeFollowerSpeed(const double & desired_interdistance,
                              const double & interdistance,
                              const double & follower_lateral_deviation,
                              const double & follower_linear_speed,
                              const double & follower_maximal_linear_speed);

  double computeFollowerSpeed(const double & desired_interdistance,
                              const double & desired_lateral_deviation,
                              const double & follower_interdistance,
                              const double & leader_interdistance,
                              const double & leader_linear_speed,
                              const double & follower_lateral_deviation,
                              const double & follower_course_deviation,
                              const double & follower_linear_speed,
                              const double & follower_maximal_linear_speed,
                              const double & desired_angular_deviation,
                              const double & yaw_rate_leader);

  double computeFollowerSpeed(const double & desired_interdistance,
                              const double & interdistance,
                              const double & leader_lateral_deviation,
                              const double & leader_course_deviation,
                              const double & leader_curvature,
                              const double & leader_linear_speed,
                              const double & leader_rear_streering_angle,
                              const double & leader_rear_sliding_angle,
                              const double & follower_lateral_deviation,
                              const double & follower_course_deviation,
                              const double & follower_curvature,
                              const double & follower_linear_speed,
                              const double & follower_rear_streering_angle,
                              const double & follower_rear_sliding_angle,
                              const double & follower_maximal_linear_speed);

  void reset();

private :


//  double clampLinearSpeed_(const double & linear_speed,
//                           const double & maximal_linear_speed);

  double offsetFreeLinearSpeed_(const double & linear_speed);

  void updateLongitudinalDeviation_(const double &interdistance,
                                    const double &desired_interdistance,
                                    const double &follower_linear_speed);

private:

  double sampling_period_;

  double KI_;
  double interdistance_error_;
  double integrated_longitudinal_deviation_;


};

}

#endif
