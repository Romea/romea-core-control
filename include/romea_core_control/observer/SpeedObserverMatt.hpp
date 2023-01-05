// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_CONTROL_OBSERVER_SPEEDOBSERVERMATT_HPP_
#define ROMEA_CORE_CONTROL_OBSERVER_SPEEDOBSERVERMATT_HPP_

// romea
#include "SpeedObserver.hpp"

namespace romea
{

class SpeedObserverMatt : public SpeedObserver
{
public:
  explicit SpeedObserverMatt(const double & samplingPeriod);

  SpeedObserverMatt(
    const double & samplingPeriod,
    const double & wheelBase);

  virtual ~SpeedObserverMatt();

  void setWheelBase(const double & wheelBase);

  const double & getWheelBase()const;

public:
  void init(
    double x,
    double y);

  void update(
    double x,
    double y,
    double linearSpeed,
    double angularSpeed);

  const double & getSpeed() const override;

  const double & getAngle() const override;

  const double & getX() const;

  const double & getY() const;

private:
  void initObserverMatt_(
    double X,
    double Y);

  void updateObserverMatt_(
    double X,
    double Y,
    double vitesse,
    double omega);

private:
  double wheelBase_;

  double XObs;
  double YObs;
  double SpeedMatt;
  double AngleMatt;
  unsigned int counter_hand_;
};

}  // namespace romea

#endif  // ROMEA_CORE_CONTROL_OBSERVER_SPEEDOBSERVERMATT_HPP_
