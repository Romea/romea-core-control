//Mathieu Deremetz // Coordonnées polaires
//romea
#include "romea_core_control/observer/SpeedObserverMatt.hpp"

//std
#include <cmath>

namespace romea {


//-----------------------------------------------------------------------------
SpeedObserverMatt::SpeedObserverMatt(const double & samplingPeriod):
  SpeedObserverMatt(samplingPeriod,0.)
{

}

//-----------------------------------------------------------------------------
SpeedObserverMatt::SpeedObserverMatt(const double & samplingPeriod,
                                     const double & wheelBase):
  SpeedObserver(samplingPeriod),
  wheelBase_(wheelBase),
  XObs(0),
  YObs(0),
  SpeedMatt(0),
  AngleMatt(0),
  counter_hand_(0)
{

}

//-----------------------------------------------------------------------------
SpeedObserverMatt::~SpeedObserverMatt()
{
}

//-----------------------------------------------------------------------------
void SpeedObserverMatt::setWheelBase(const double & wheelBase)
{
  wheelBase_=wheelBase;
}

//-----------------------------------------------------------------------------
const double & SpeedObserverMatt::getWheelBase()const
{
  return wheelBase_;
}




//-----------------------------------------------------------------------------
void SpeedObserverMatt::update(double x,
                               double y,
                               double linearSpeed,
                               double angularspeed)
{

  //std::cout << "update "<<std::endl;
  if(!is_initialized_)
  {
    //std::cout << "update 1"<<std::endl;

    initObserverMatt_(x,y);
    is_initialized_=true;
  }
  else
  {
    //std::cout << "update 2"<<std::endl;

    updateObserverMatt_(x,y,linearSpeed,angularspeed);
  }
}


//-----------------------------------------------------------------------------
void SpeedObserverMatt::initObserverMatt_(double X,
                                          double Y)
{
  XObs=X;
  YObs=Y;
  SpeedMatt=AngleMatt=0;
  counter_hand_ =0;
  std::cout << "var" << " " << X <<" "<< Y <<" "<< SpeedMatt <<" "<< AngleMatt <<std::endl;
}

//-----------------------------------------------------------------------------
const double & SpeedObserverMatt::getSpeed() const
{
  return SpeedMatt;
}

//-----------------------------------------------------------------------------
const double & SpeedObserverMatt::getAngle() const
{
  return AngleMatt;
}


//-----------------------------------------------------------------------------
void SpeedObserverMatt::updateObserverMatt_(double X,
                                            double Y,
                                            double vitesse,
                                            double omega)
{
  //const double &wheelbase_= wheelBase_;
  const double &sampling_period_ = samplingPeriod_;

  //std::cout << "var" << " " << X <<" "<< Y <<" "<< vitesse <<" "<< omega <<std::endl;
  double rho=X;
  double angle=Y;

  int N=20;
  // Definition gain observation

  double Kx=50;
  double Ky=50;

  double Kvitesse1=50; //
  double Kvitesse2=50; //
  double Kangle1=500;	//100
  double Kangle2=500;	//100

  //Definition matrices

  // -- Calcul derive
  for(int i=1;i<N+1;i++)
  {
    // Derive partielle
    double DXDv=std::cos(AngleMatt);
    double DYDv=-std::sin(AngleMatt);
    double DXDa=SpeedMatt*-std::sin(AngleMatt)/rho;
    double DYDa=-SpeedMatt*std::cos(AngleMatt)/rho;
    //    double DXDv=std::cos(AngleMatt);
    //    double DYDv=std::sin(AngleMatt);
    //    double DXDa=SpeedMatt*-std::sin(AngleMatt);
    //    double DYDa=SpeedMatt*std::cos(AngleMatt);

    // Fonction f
    double Xdot=-vitesse*std::cos(angle)+SpeedMatt*std::cos(AngleMatt);
    double Ydot=((-SpeedMatt*std::sin(AngleMatt))/rho)+omega;
    //    double Xdot=-vitesse+SpeedMatt*std::cos(AngleMatt)+omega*YObs;
    //    double Ydot=SpeedMatt*std::sin(AngleMatt)+omega*XObs;

    // MaJ Beta:
    double dotSpeedMatt, dotAngleMatt;

    dotSpeedMatt=Kvitesse1*(DXDv*(X-XObs))+Kvitesse2*(DYDv*(Y-YObs));
    dotAngleMatt=Kangle1*(DXDa*(X-XObs))+Kangle2*(DYDa*(Y-YObs));

    XObs	+=sampling_period_/N*(Xdot + Kx*(X-XObs));
    YObs	+=sampling_period_/N*(Ydot + Ky*(Y-YObs));

    SpeedMatt+=sampling_period_/N*dotSpeedMatt;
    AngleMatt+=sampling_period_/N*dotAngleMatt;

    if (SpeedMatt<0.2) AngleMatt=0;

  }
//  std::cout << "pos-reel" << " " << X <<" "<< Y <<std::endl;
//  std::cout << "pos-esti" << " " << XObs <<" "<< YObs <<std::endl;

  if (SpeedMatt>2)
  {
    SpeedMatt=2;
  }


  //  if (SpeedMatt<0.3 || vitesse==0)
  //  {
  //    AngleMatt=0;
  //  }

  if(counter_hand_<2)
  {
    XObs=X;
    YObs=Y;
    SpeedMatt=0;
    AngleMatt=0;
  }
  counter_hand_++;
}

//-----------------------------------------------------------------------------
const double & SpeedObserverMatt::getX() const
{
   return XObs;
}

//-----------------------------------------------------------------------------
const double & SpeedObserverMatt::getY() const
{
  return YObs;
}


}
