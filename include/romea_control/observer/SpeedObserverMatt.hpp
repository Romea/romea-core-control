#ifndef _romea_MattSpeedObserver_hpp
#define _romea_MattSpeedObserver_hpp

//romea
#include "SpeedObserver.hpp"

namespace romea {

class SpeedObserverMatt : public SpeedObserver
{

public:

  SpeedObserverMatt(const double & samplingPeriod);

  SpeedObserverMatt(const double & samplingPeriod,
                    const double & wheelBase);

  virtual ~SpeedObserverMatt();

  void setWheelBase(const double & wheelBase);

  const double & getWheelBase()const;

public:


  void init(double x,
            double y);

  void update(double x,
              double y,
              double linearSpeed,
              double angularSpeed);

  virtual const double & getSpeed() const override;

  virtual const double & getAngle() const override;

  const double & getX() const;

  const double & getY() const;

private :

  void initObserverMatt_(double X,
                         double Y);

  void updateObserverMatt_(double X,
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

}
#endif
