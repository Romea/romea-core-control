#include "romea_control/observer/SpeedObserver.hpp"

namespace romea {

//-----------------------------------------------------------------------------
SpeedObserver::SpeedObserver(const double & samplingPeriod):
  samplingPeriod_(samplingPeriod),
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
  return {getSpeed(),getAngle()};
}

//-----------------------------------------------------------------------------
void SpeedObserver::reset()
{
  is_initialized_=false;
}

}
