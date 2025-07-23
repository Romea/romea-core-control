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

#ifndef ROMEA_CORE_CONTROL__OBSERVER__SPEEDOBSERVERMATT_HPP_
#define ROMEA_CORE_CONTROL__OBSERVER__SPEEDOBSERVERMATT_HPP_

// romea
#include "SpeedObserver.hpp"

namespace romea::core
{

class SpeedObserverMatt : public SpeedObserver
{
public:
  explicit SpeedObserverMatt(double samplingPeriod);

  SpeedObserverMatt(double samplingPeriod, double wheelBase);

  virtual ~SpeedObserverMatt();

  void setWheelBase(double wheelBase);

  double getWheelBase() const;

public:
  void init(double x, double y);

  void update(double x, double y, double linearSpeed, double angularSpeed);

  double getSpeed() const override;

  double getAngle() const override;

  double getX() const;

  double getY() const;

private:
  void initObserverMatt_(double X, double Y);

  void updateObserverMatt_(double X, double Y, double vitesse, double omega);

private:
  double wheelBase_;

  double XObs;
  double YObs;
  double SpeedMatt;
  double AngleMatt;
  unsigned int counter_hand_;
};

}  // namespace romea::core

#endif  // ROMEA_CORE_CONTROL__OBSERVER__SPEEDOBSERVERMATT_HPP_
