#ifndef _romea_SlidingObserverCinematic_hpp__
#define _romea_SlidingObserverCinematic_hpp__

//std
#include <iostream>

//romea
#include "../FrontRearData.hpp"

namespace romea {

class SlidingObserverCinematic
{
public:

  SlidingObserverCinematic(const double & samplingPeriod);

  virtual ~SlidingObserverCinematic()=default;

public :

  FrontRearData getSlidingAngles()const;

  virtual const double & getFrontSlidingAngle() const =0;

  virtual const double & getRearSlidingAngle() const=0;

  virtual void reset();

protected :

  double samplingPeriod_;
  bool is_initialized_;

};

}
#endif
