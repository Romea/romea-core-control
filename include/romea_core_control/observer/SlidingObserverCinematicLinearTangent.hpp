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


#ifndef ROMEA_CORE_CONTROL__OBSERVER__SLIDINGOBSERVERCINEMATICLINEARTANGENT_HPP_
#define ROMEA_CORE_CONTROL__OBSERVER__SLIDINGOBSERVERCINEMATICLINEARTANGENT_HPP_

// eigen
#include <Eigen/Dense>


// romea
#include "romea_core_common/signal/FirstOrderButterworth.hpp"
#include "romea_core_control/observer/SlidingObserverCinematic.hpp"

namespace romea
{


class SlidingObserverCinematicLinearTangent : public SlidingObserverCinematic
{
public:
  struct Parameters
  {
    double lateralDeviationGain;
    double courseDeviationGain;
    double lateralDeviationFilterWeight;
    double courseDeviationFilterWeight;
    double frontSlidingAngleFilterWeight;
    double rearSlidingAngleFilterWeight;
  };

public:
  SlidingObserverCinematicLinearTangent(
    const double & samplingPeriod,
    const double & wheelBase,
    const Parameters & parameters);

  virtual ~SlidingObserverCinematicLinearTangent() = default;

public:
  void update(
    const double & lateralDeviation,
    const double & courseDeviation,
    const double & curvature,
    const double & linearSpeed,
    const double & frontSteeringAngle,
    const double & rearSteeringAngle);

  const double & getFrontSlidingAngle() const override;

  const double & getRearSlidingAngle() const override;

  const double & getLateralDeviation()const;

  const double & getCourseDeviation()const;

  void initObserver_(
    const double & ElatM,
    const double & EcapM);

private:
  /**
   * Compute sliding
   * @param[in] lateral_deviation : lateral deviation in meters, positive on the right
   * @param[in] cap_deviation : cap deviation in radian, counterclockwise
   * @param[in] speed : linear speed of the robot
   * @param[in] front_steering : in radian, counterclockwise
   * @param[in] curvature at the point of the trajectory which is nearest to the robot
   * @param[in] rear_steering : in radian, counterclockwise
  **/
  bool computeSliding_(
    const double & lateral_deviation,
    const double & cap_deviation,
    const double & speed,
    const double & front_steering,
    const double & curvature,
    const double & rear_steering);

  bool computeSliding2_(
    const double & lateral_deviation,
    const double & cap_deviation,
    const double & speed,
    const double & front_steering,
    const double & curvature,
    const double & rear_steering);

  /**
   * evolution of sliding observer
   * @param[in] lateral_deviation : lateral deviation in meters, positive on the right
   * @param[in] cap_deviation : cap deviation in radian, counterclockwise
   * @param[in] speed : linear speed of the robot
   * @param[in] front_steering : in radian, counterclockwise
   * @param[in] curvature at the point of the trajectory which is nearest to the robot
   * @param[in] rear_steering : in radian, counterclockwise
  **/
  void evolution_(
    const double & lateral_deviation,
    const double & cap_deviation,
    const double & speed,
    const double & front_steering,
    const double & curvature,
    const double & rear_steering);

  void evolution2_(
    const double & lateral_deviation,
    const double & cap_deviation,
    const double & speed,
    const double & front_steering,
    const double & curvature,
    const double & rear_steering);

private:
  double wheelBase_;

  double betaR;
  double betaF;
  double betaRF;
  double betaFF;
  double Elat4;
  double Ecap4;
  double Elat_av;
  double Ecap_av;
  double ElatDeriv;
  double EcapDeriv;
  unsigned int counter_;

  Eigen::Matrix2d G_, B_, invB_;
  Eigen::Vector2d A_, Y_, K_, X_;
  FirstOrderButterworth lateral_deviation_drift_f_;
  FirstOrderButterworth cap_deviation_drift_f_;
  FirstOrderButterworth betaR_f_;
  FirstOrderButterworth betaF_f_;
};

}  // namespace romea

#endif  // ROMEA_CORE_CONTROL__OBSERVER__SLIDINGOBSERVERCINEMATICLINEARTANGENT_HPP_
