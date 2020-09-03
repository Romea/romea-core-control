#include "romea_control/observer/SlidingObserverCinematic.hpp"

namespace romea {

//-----------------------------------------------------------------------------
SlidingObserverCinematic::SlidingObserverCinematic(const double & samplingPeriod):
  samplingPeriod_(samplingPeriod),
  is_initialized_(false)
{

}

//-----------------------------------------------------------------------------
FrontRearData SlidingObserverCinematic::getSlidingAngles()const
{
  return {getFrontSlidingAngle(),getRearSlidingAngle()};
}

//-----------------------------------------------------------------------------
void SlidingObserverCinematic::reset()
{
  is_initialized_=false;
}

}
