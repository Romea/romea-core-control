// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#include "romea_core_control/observer/SlidingObserverCinematic.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
SlidingObserverCinematic::SlidingObserverCinematic(const double & samplingPeriod)
: samplingPeriod_(samplingPeriod),
  is_initialized_(false)
{

}

//-----------------------------------------------------------------------------
FrontRearData SlidingObserverCinematic::getSlidingAngles()const
{
  return {getFrontSlidingAngle(), getRearSlidingAngle()};
}

//-----------------------------------------------------------------------------
void SlidingObserverCinematic::reset()
{
  is_initialized_ = false;
}

}  // namespace romea
