#ifndef _romea_SlidingObserverKinematicLyapunov_hpp
#define _romea_SlidingObserverKinematicLyapunov_hpp

//romea
#include "SlidingObserverCinematic.hpp"

namespace romea {

class SlidingObserverCinematicLyapounov : public SlidingObserverCinematic
{
public :

  struct Parameters
  {
    double xDeviationGain;
    double yDeviationGain;
    double courseDeviationGain;
    double frontSlidingAngleGain;
    double rearSlidingAngleGain;
  };

public:

  SlidingObserverCinematicLyapounov(const double & samplingPeriod,
                                    const double & wheelBase,
                                    const Parameters & parameters);

  virtual ~SlidingObserverCinematicLyapounov()=default;


public:

  void update(double x,
              double y,
              double course,
              double linearSpeed,
              double frontSteeringAngle,
              double rearSteeringAngle);

  virtual const double & getFrontSlidingAngle() const override;

  virtual const double & getRearSlidingAngle() const override;

  const double & getX() const;

  const double & getY() const;

  const double & getTheta() const;

private :

  void initObserverHandbooks_(double X,
                              double Y,
                              double Theta);

  void updateObserverHandbooks_(double X,
                                double Y,
                                double Theta,
                                double deltaF,
                                double deltaR,
                                double vitesse);

private:

  double wheelBase_;

  double XObs;
  double YObs;
  double ThetaObs;
  double BetaFHand;
  double BetaRHand;
  unsigned int counter_hand_;

  double Kone1_;
  double Kone2_;
  double Kone3_;
  double Ktwo1_;
  double Ktwo2_;

};

}
#endif
