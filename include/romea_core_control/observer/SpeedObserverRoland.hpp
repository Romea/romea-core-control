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

namespace romea
{
namespace core
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

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_CONTROL__OBSERVER__SPEEDOBSERVERROLAND_HPP_
