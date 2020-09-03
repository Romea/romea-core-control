#ifndef _romea_LeaderSpeedObserver_hpp
#define _romea_LeaderSpeedObserver_hpp

namespace romea {

class SpeedObserverRoland
{
public:

  SpeedObserverRoland(const double & sample_period);

  SpeedObserverRoland(const double & sample_period,
                      const double & kd);


  double update(const double & longitudinal_deviation,
                const double & course_deviation,
                const double & follower_linear_speed,
                const double & follower_angular_speed);

  double update(const double & longitudinal_deviation,
                const double & course_deviation,
                const double & follower_linear_speed);

  void reset();

public:

  static const double DEFAULT_KD;

private:

  double sampling_period_;
  double kd_;

  double longitudinal_deviation_obs_;
  double leader_linear_speed_obs_;

  bool is_initialized_;
};
}

#endif
