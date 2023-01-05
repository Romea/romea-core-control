// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_CONTROL__OBSERVER__SLIDINGOBSERVERCINEMATIC_HPP_
#define ROMEA_CORE_CONTROL__OBSERVER__SLIDINGOBSERVERCINEMATIC_HPP_

// std
#include <iostream>

// romea
#include "romea_core_control/FrontRearData.hpp"

namespace romea
{

class SlidingObserverCinematic
{
public:
  explicit SlidingObserverCinematic(const double & samplingPeriod);

  virtual ~SlidingObserverCinematic() = default;

public:
  FrontRearData getSlidingAngles()const;

  virtual const double & getFrontSlidingAngle() const = 0;

  virtual const double & getRearSlidingAngle() const = 0;

  virtual void reset();

protected:
  double samplingPeriod_;
  bool is_initialized_;

};

}
#endif   // ROMEA_CORE_CONTROL__OBSERVER__SLIDINGOBSERVERCINEMATIC_HPP_
