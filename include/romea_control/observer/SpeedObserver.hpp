#ifndef _romea_SpeedObserver_hpp__
#define _romea_SpeedObserver_hpp__

//romea
#include "../SpeedAngleData.hpp"

namespace romea {

class SpeedObserver
{
public:

  SpeedObserver(const double & samplingPeriod);

  virtual ~SpeedObserver();

public :

  SpeedAngleData getSpeedAngle()const;

  virtual const double & getSpeed() const =0;

  virtual const double & getAngle() const=0;

  virtual void reset();

protected :

  double samplingPeriod_;
  bool is_initialized_;

};

}
#endif
