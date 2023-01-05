// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#include "romea_core_control/observer/SpeedObserver.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
SpeedObserver::SpeedObserver(const double & samplingPeriod)
: samplingPeriod_(samplingPeriod),
  is_initialized_(false)
{
}

//-----------------------------------------------------------------------------
SpeedObserver::~SpeedObserver()
{
}

//-----------------------------------------------------------------------------
SpeedAngleData SpeedObserver::getSpeedAngle()const
{
  return {getSpeed(), getAngle()};
}

//-----------------------------------------------------------------------------
void SpeedObserver::reset()
{
  is_initialized_ = false;
}

}  // namespace romea
