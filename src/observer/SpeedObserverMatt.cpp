// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Mathieu Deremetz // Coordonnées polaires
// romea
#include "romea_core_control/observer/SpeedObserverMatt.hpp"

// std
#include <cmath>

namespace romea::core
{

//-----------------------------------------------------------------------------
SpeedObserverMatt::SpeedObserverMatt(double samplingPeriod) : SpeedObserverMatt(samplingPeriod, 0.)
{
}

//-----------------------------------------------------------------------------
SpeedObserverMatt::SpeedObserverMatt(double samplingPeriod, double wheelBase)
: SpeedObserver(samplingPeriod),
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
void SpeedObserverMatt::setWheelBase(double wheelBase)
{
  wheelBase_ = wheelBase;
}

//-----------------------------------------------------------------------------
double SpeedObserverMatt::getWheelBase() const
{
  return wheelBase_;
}

//-----------------------------------------------------------------------------
void SpeedObserverMatt::update(double x, double y, double linearSpeed, double angularspeed)
{
  if (!is_initialized_) {
    is_initialized_ = true;
    initObserverMatt_(x, y);
  } else {
    updateObserverMatt_(x, y, linearSpeed, angularspeed);
  }
}

//-----------------------------------------------------------------------------
void SpeedObserverMatt::initObserverMatt_(double X, double Y)
{
  XObs = X;
  YObs = Y;
  SpeedMatt = AngleMatt = 0;
  counter_hand_ = 0;
}

//-----------------------------------------------------------------------------
double SpeedObserverMatt::getSpeed() const
{
  return SpeedMatt;
}

//-----------------------------------------------------------------------------
double SpeedObserverMatt::getAngle() const
{
  return AngleMatt;
}

//-----------------------------------------------------------------------------
void SpeedObserverMatt::updateObserverMatt_(double X, double Y, double vitesse, double omega)
{
  // doublewheelbase_= wheelBase_;
  double sampling_period_ = samplingPeriod_;

  // std::cout << "var" << " " << X <<" "<< Y <<" "<< vitesse <<" "<< omega <<std::endl;
  double rho = X;
  double angle = Y;

  int N = 20;
  // Definition gain observation

  double Kx = 50;
  double Ky = 50;

  double Kvitesse1 = 50;  //
  double Kvitesse2 = 50;  //
  double Kangle1 = 500;   // 100
  double Kangle2 = 500;   // 100

  // Definition matrices

  // -- Calcul derive
  for (int i = 1; i < N + 1; i++) {
    // Derive partielle
    double DXDv = std::cos(AngleMatt);
    double DYDv = -std::sin(AngleMatt);
    double DXDa = SpeedMatt * -std::sin(AngleMatt) / rho;
    double DYDa = -SpeedMatt * std::cos(AngleMatt) / rho;
    //    double DXDv=std::cos(AngleMatt);
    //    double DYDv=std::sin(AngleMatt);
    //    double DXDa=SpeedMatt*-std::sin(AngleMatt);
    //    double DYDa=SpeedMatt*std::cos(AngleMatt);

    // Fonction f
    double Xdot = -vitesse * std::cos(angle) + SpeedMatt * std::cos(AngleMatt);
    double Ydot = ((-SpeedMatt * std::sin(AngleMatt)) / rho) + omega;
    //    double Xdot=-vitesse+SpeedMatt*std::cos(AngleMatt)+omega*YObs;
    //    double Ydot=SpeedMatt*std::sin(AngleMatt)+omega*XObs;

    // MaJ Beta:
    double dotSpeedMatt, dotAngleMatt;

    dotSpeedMatt = Kvitesse1 * (DXDv * (X - XObs)) + Kvitesse2 * (DYDv * (Y - YObs));
    dotAngleMatt = Kangle1 * (DXDa * (X - XObs)) + Kangle2 * (DYDa * (Y - YObs));

    XObs += sampling_period_ / N * (Xdot + Kx * (X - XObs));
    YObs += sampling_period_ / N * (Ydot + Ky * (Y - YObs));

    SpeedMatt += sampling_period_ / N * dotSpeedMatt;
    AngleMatt += sampling_period_ / N * dotAngleMatt;

    if (SpeedMatt < 0.2) {
      AngleMatt = 0;
    }
  }
  //  std::cout << "pos-reel" << " " << X <<" "<< Y <<std::endl;
  //  std::cout << "pos-esti" << " " << XObs <<" "<< YObs <<std::endl;

  if (SpeedMatt > 2) {
    SpeedMatt = 2;
  }

  //  if (SpeedMatt<0.3 || vitesse==0)
  //  {
  //    AngleMatt=0;
  //  }

  if (counter_hand_ < 2) {
    XObs = X;
    YObs = Y;
    SpeedMatt = 0;
    AngleMatt = 0;
  }
  counter_hand_++;
}

//-----------------------------------------------------------------------------
double SpeedObserverMatt::getX() const
{
  return XObs;
}

//-----------------------------------------------------------------------------
double SpeedObserverMatt::getY() const
{
  return YObs;
}

}  // namespace romea::core
