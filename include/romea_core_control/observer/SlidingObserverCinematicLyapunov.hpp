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


#ifndef ROMEA_CORE_CONTROL__OBSERVER__SLIDINGOBSERVERCINEMATICLYAPUNOV_HPP_
#define ROMEA_CORE_CONTROL__OBSERVER__SLIDINGOBSERVERCINEMATICLYAPUNOV_HPP_

// romea
#include "SlidingObserverCinematic.hpp"

namespace romea
{
namespace core
{

class SlidingObserverCinematicLyapunov : public SlidingObserverCinematic
{
public:
  struct Parameters
  {
    double xDeviationGain;
    double yDeviationGain;
    double courseDeviationGain;
    double frontSlidingAngleGain;
    double rearSlidingAngleGain;
  };

public:
  SlidingObserverCinematicLyapunov(
    const double & samplingPeriod,
    const double & wheelBase,
    const Parameters & parameters);

  virtual ~SlidingObserverCinematicLyapunov() = default;

public:
  void update(
    double x,
    double y,
    double course,
    double linearSpeed,
    double frontSteeringAngle,
    double rearSteeringAngle);

  const double & getFrontSlidingAngle() const override;

  const double & getRearSlidingAngle() const override;

  const double & getX() const;

  const double & getY() const;

  const double & getTheta() const;

  void initObserverHandbooks_(
    double X,
    double Y,
    double Theta);

private:
  void updateObserverHandbooks_(
    double X,
    double Y,
    double Theta,
    double deltaF,
    double deltaR,
    double vitesse);

private:
  double wheelBase_;

  double XObs;
  double YObs;
  double ThetaObs;
  double BetaFHand;
  double BetaRHand;
  unsigned int counter_hand_;

  double Kone1_;
  double Kone2_;
  double Kone3_;
  double Ktwo1_;
  double Ktwo2_;
};

}  // namespace core
}  // namespace romea

#endif  //  ROMEA_CORE_CONTROL__OBSERVER__SLIDINGOBSERVERCINEMATICLYAPUNOV_HPP_
