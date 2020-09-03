//romea
#include "romea_control/observer/SlidingObserverCinematicLyapounov.hpp"

//std
#include <cmath>

namespace romea {

//-----------------------------------------------------------------------------
SlidingObserverCinematicLyapounov::SlidingObserverCinematicLyapounov(const double & samplingPeriod,
                                                                     const double & wheelBase,
                                                                     const Parameters & parameters):
  SlidingObserverCinematic(samplingPeriod),
  wheelBase_(wheelBase),
  XObs(0),
  YObs(0),
  ThetaObs(0),
  BetaFHand(0),
  BetaRHand(0),
  counter_hand_(0),
  Kone1_(parameters.xDeviationGain),
  Kone2_(parameters.yDeviationGain),
  Kone3_(parameters.courseDeviationGain),
  Ktwo1_(parameters.frontSlidingAngleGain),
  Ktwo2_(parameters.rearSlidingAngleGain)
{

}


//-----------------------------------------------------------------------------
void SlidingObserverCinematicLyapounov::update(double x,
                                               double y,
                                               double course,
                                               double linearSpeed,
                                               double frontSteeringAngle,
                                               double rearSteeringAngle)
{
  if(!is_initialized_)
  {
    initObserverHandbooks_(x,y,course);
    is_initialized_=true;
  }
  else
  {
    updateObserverHandbooks_(x,y,course,frontSteeringAngle,rearSteeringAngle,linearSpeed);
  }
}


//-----------------------------------------------------------------------------
void SlidingObserverCinematicLyapounov::initObserverHandbooks_(double X,
                                                               double Y,
                                                               double Theta)
{
  XObs=X;
  YObs=Y;
  ThetaObs=Theta;
  BetaFHand=BetaRHand=0;
  counter_hand_ =0;
}

//-----------------------------------------------------------------------------
const double & SlidingObserverCinematicLyapounov::getFrontSlidingAngle() const
{
  return BetaFHand;
}

//-----------------------------------------------------------------------------
const double & SlidingObserverCinematicLyapounov::getRearSlidingAngle() const
{
  return BetaRHand;
}


//-----------------------------------------------------------------------------
void SlidingObserverCinematicLyapounov::updateObserverHandbooks_(double X,
                                                                 double Y,
                                                                 double Theta,
                                                                 double deltaF,
                                                                 double deltaR,
                                                                 double vitesse)
{
  const double &wheelbase_= wheelBase_;
  const double &sampling_period_ = samplingPeriod_;

  if((ThetaObs-Theta)>M_PI/2) ThetaObs-=M_PI;
  if((ThetaObs-Theta)<-M_PI/2) ThetaObs+=M_PI;

  int N=10;
  // Definition gin observation
  //~ double Kone1=-7;
  //~ double Kone2=-7;
  //~ double Kone3=-7;
//  double gaingene=3; //8
//  double Kone1=-gaingene;
//  double Kone2=-gaingene;
//  double Kone3=-3; //2

  //BetaFHand=betaFF;
  //BetaRHand=betaR;
  //~ double Ktwo1=-.5; //.7
  //~ double Ktwo2=-.5;	// .7
//  double Ktwo1=-1.8; //.4
//  double Ktwo2=-1.22;	// .4
  //~ double Ktwo1=0;
  //~ double Ktwo1=0;
  //~ double Ktwo2=0;
  //~ Theta+=0.1*vitesse*std::tan(deltaF)/wheelbase_;
  //~ X-=0.1*vitesse*std::cos(Theta);
  //~ Y-=0.1*vitesse*std::sin(Theta);

  //Definition matrices
  // -- Calcul derive
  for(int i=1;i<N+1;i++)
  {
    // Derive partielle
    double B11=0;
    double B21=0;
    double B31=(vitesse/wheelbase_)*std::cos(deltaR+BetaRHand)*(1+std::tan(deltaF+BetaFHand)*std::tan(deltaF+BetaFHand));
    double B12=-vitesse*std::sin(Theta+BetaRHand+deltaR);
    double B22=vitesse*std::cos(Theta+BetaRHand+deltaR);
    double B32=-(vitesse/wheelbase_)*std::cos(deltaR+BetaRHand)*(1+std::tan(deltaR+BetaRHand)*std::tan(deltaR+BetaRHand))-(vitesse/wheelbase_)*std::sin(BetaRHand+deltaR)*(std::tan(deltaF+BetaFHand)-std::tan(deltaR+BetaRHand));

    // Fonction f
    double F1=vitesse*std::cos(Theta+BetaRHand+deltaR);
    double F2=vitesse*std::sin(Theta+BetaRHand+deltaR);
    double F3=vitesse*std::cos(BetaRHand+deltaR)*(std::tan(deltaF+BetaFHand)-std::tan(deltaR+BetaRHand))/wheelbase_;

    XObs	+=sampling_period_/N*(F1 + Kone1_*(XObs-X));
    YObs	+=sampling_period_/N*(F2 + Kone2_*(YObs-Y));
    ThetaObs+=sampling_period_/N*(F3 + Kone3_*(ThetaObs-Theta));

    // MaJ Beta:
    double dotBetaR, dotBetaF;

    //~ if (fabs(B12*(XObs-X)+B22*(YObs-Y)+B32*(ThetaObs-Theta))<0.001) {Ktwo1=Ktwo2=-40;}
    //~ else {Ktwo1=Ktwo2=-4;}
    //~
    dotBetaF=Ktwo1_*(B11*(XObs-X)+B21*(YObs-Y)+B31*(ThetaObs-Theta));
    dotBetaR=Ktwo2_*(B12*(XObs-X)+B22*(YObs-Y)+B32*(ThetaObs-Theta));

    //~ if (fabs(B12)>fabs(B22))
    //~ {dotBetaR=Ktwo2*(B12*(XObs-X)+B22*(YObs-Y)+B32*(ThetaObs-Theta));}
    //~ else
    //~ {dotBetaR=Ktwo2*(B12*(XObs-X)+B22*(YObs-Y)+B32*(ThetaObs-Theta));}
    //~

    BetaFHand+=sampling_period_/N*dotBetaF;
    BetaRHand+=sampling_period_/N*dotBetaR;
  }
  if(counter_hand_<10)
  {
    XObs=X;
    YObs=Y;
    ThetaObs=Theta;
    BetaFHand=BetaRHand=0;
  }
  counter_hand_++;
}

//-----------------------------------------------------------------------------
const double & SlidingObserverCinematicLyapounov::getX() const
{
  return XObs;
}

//-----------------------------------------------------------------------------
const double & SlidingObserverCinematicLyapounov::getY() const
{
  return YObs;
}

//-----------------------------------------------------------------------------
const double & SlidingObserverCinematicLyapounov::getTheta() const
{
  return ThetaObs;
}

}
