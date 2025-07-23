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

#ifndef ROMEA_CORE_CONTROL__OBSERVER__SPEEDOBSERVERROLAND_HPP_
#define ROMEA_CORE_CONTROL__OBSERVER__SPEEDOBSERVERROLAND_HPP_

namespace romea::core
{

class SpeedObserverRoland
{
public:
  explicit SpeedObserverRoland();

  SpeedObserverRoland(double kd);

  double update(
    double delta_time,
    double longitudinal_deviation,
    double course_deviation,
    double follower_linear_speed,
    double follower_angular_speed);

  double update(
    double delta_time,
    double longitudinal_deviation,
    double course_deviation,
    double follower_linear_speed);

  void reset();

public:
  static const double DEFAULT_KD;

private:
  double kd_;

  double longitudinal_deviation_obs_;
  double leader_linear_speed_obs_;

  bool is_initialized_;
};

}  // namespace romea::core

#endif  // ROMEA_CORE_CONTROL__OBSERVER__SPEEDOBSERVERROLAND_HPP_
