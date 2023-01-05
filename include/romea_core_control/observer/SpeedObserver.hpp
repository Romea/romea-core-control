// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_CONTROL__OBSERVER__SPEEDOBSERVER_HPP_
#define ROMEA_CORE_CONTROL__OBSERVER__SPEEDOBSERVER_HPP_

// std
#include <iostream>

// romea
#include "romea_core_control/SpeedAngleData.hpp"

namespace romea
{

class SpeedObserver
{
public:
  explicit SpeedObserver(const double & samplingPeriod);

  virtual ~SpeedObserver();

public:
  SpeedAngleData getSpeedAngle()const;

  virtual const double & getSpeed() const = 0;

  virtual const double & getAngle() const = 0;

  virtual void reset();

protected:
  double samplingPeriod_;
  bool is_initialized_;
};

std::ostream & operator<<(std::ostream & os, const SpeedObserver & observer);

}  // namespace romea

#endif  // ROMEA_CORE_CONTROL__OBSERVER__SPEEDOBSERVER_HPP_
