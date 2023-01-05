// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_CONTROL__OBSERVER__SPEEDOBSERVERROLAND_HPP_
#define ROMEA_CORE_CONTROL__OBSERVER__SPEEDOBSERVERROLAND_HPP_

namespace romea
{

class SpeedObserverRoland
{
public:
  explicit SpeedObserverRoland(const double & sample_period);

  SpeedObserverRoland(
    const double & sample_period,
    const double & kd);


  double update(
    const double & longitudinal_deviation,
    const double & course_deviation,
    const double & follower_linear_speed,
    const double & follower_angular_speed);

  double update(
    const double & longitudinal_deviation,
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

}  // namespace romea

#endif  // ROMEA_CORE_CONTROL__OBSERVER__SPEEDOBSERVERROLAND_HPP_
