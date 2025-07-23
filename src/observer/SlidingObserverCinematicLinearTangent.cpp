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


// std
#include <cmath>

// romea
#include "romea_core_control/observer/SlidingObserverCinematicLinearTangent.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
SlidingObserverCinematicLinearTangent::SlidingObserverCinematicLinearTangent(
  const double & samplingPeriod,
  const double & wheelBase,
  const Parameters & parameters)
: SlidingObserverCinematic(samplingPeriod),
  wheelBase_(wheelBase),
  betaR(0),
  betaF(0),
  betaRF(0),
  betaFF(0),
  Elat4(0),
  Ecap4(0),
  Elat_av(0),
  Ecap_av(0),
  ElatDeriv(0),
  EcapDeriv(0),
  counter_(0),
  G_(Eigen::Matrix2d::Zero()),
  B_(Eigen::Matrix2d::Zero()),
  invB_(Eigen::Matrix2d::Zero()),
  A_(Eigen::Vector2d::Zero()),
  Y_(Eigen::Vector2d::Zero()),
  K_(Eigen::Vector2d::Zero()),
  X_(Eigen::Vector2d::Zero()),
  lateral_deviation_drift_f_(parameters.lateralDeviationFilterWeight),
  cap_deviation_drift_f_(parameters.courseDeviationFilterWeight),
  betaR_f_(parameters.rearSlidingAngleFilterWeight),
  betaF_f_(parameters.frontSlidingAngleFilterWeight)
{
  G_(0, 0) = parameters.lateralDeviationGain;
  G_(1, 1) = parameters.courseDeviationGain;
}


//-----------------------------------------------------------------------------
void SlidingObserverCinematicLinearTangent::update(
  const double & lateralDeviation,
  const double & courseDeviation,
  const double & curvature,
  const double & linearSpeed,
  const double & frontSteeringAngle,
  const double & rearSteeringAngle)
{
  if (!is_initialized_ || std::abs(linearSpeed) < 0.2) {
    initObserver_(lateralDeviation, courseDeviation);
    lateral_deviation_drift_f_.reset();
    cap_deviation_drift_f_.reset();
    betaR_f_.reset();
    betaF_f_.reset();
    is_initialized_ = true;
  } else {
    computeSliding_(
      lateralDeviation,
      courseDeviation,
      linearSpeed,
      frontSteeringAngle,
      curvature,
      rearSteeringAngle);

    evolution_(
      lateralDeviation,
      courseDeviation,
      linearSpeed,
      frontSteeringAngle,
      curvature,
      rearSteeringAngle);
  }
}


//-----------------------------------------------------------------------------
double SlidingObserverCinematicLinearTangent::getFrontSlidingAngle() const
{
  return betaFF;
}

//-----------------------------------------------------------------------------
double SlidingObserverCinematicLinearTangent::getRearSlidingAngle() const
{
  return betaRF;
}

//-----------------------------------------------------------------------------
void SlidingObserverCinematicLinearTangent::initObserver_(
  const double & ElatM,
  const double & EcapM)
{
  Elat4 = Elat_av = ElatM;
  Ecap4 = Ecap_av = EcapM;
  ElatDeriv = 0;
  EcapDeriv = 0;
  betaR = 0;
  betaF = 0;
  betaFF = betaRF = 0;
  counter_ = 0;
}

//-----------------------------------------------------------------------------
bool SlidingObserverCinematicLinearTangent::computeSliding_(
  const double & ElatM,
  const double & EcapM,
  const double & vitesse,
  const double & delta,
  const double & courb,
  const double & d_AR)
{
  const double & wheelbase_ = wheelBase_;
  const double & sampling_period_ = samplingPeriod_;

  //  Eigen::Matrix2d G,B,invB;
  //  Eigen::Vector2d A,Y,K,X;

  //  G(0,0) = -4;
  //  G(0,1) = 0;
  //  G(1,0) = 0;
  //  G(1,1) = -2;
  /* G(0,0) = -10;  // Gain oservateur
    G(0,1) = 0;
    G(1,0) = 0;
    G(1,1) = -5;*/

  A_(0) = vitesse * std::sin(Ecap4 + d_AR);
  A_(1) = vitesse * std::cos(d_AR) * (std::tan(delta) - std::tan(d_AR)) / wheelbase_ - vitesse *
    (courb * std::cos(Ecap4 + d_AR) / (1 - courb * (Elat4)));

  B_(0, 0) = vitesse * std::cos(Ecap4 + d_AR);
  B_(0, 1) = 0;
  B_(
    1,
    0) = vitesse * (courb * std::sin(Ecap4 + d_AR) / (1 - courb * Elat4)) - vitesse * (std::cos(
      d_AR) + std::tan(delta) * std::sin(d_AR)) / wheelbase_;
  B_(1, 1) = (vitesse * std::cos(d_AR) / wheelbase_) * (1 + pow(std::tan(delta), 2));

  // Derivation Ecart lateral et angulaire & Filtrage
  ElatDeriv = (ElatM - Elat_av) / sampling_period_;
  EcapDeriv = (EcapM - Ecap_av) / sampling_period_;

  ElatDeriv = lateral_deviation_drift_f_.update(ElatDeriv);
  EcapDeriv = cap_deviation_drift_f_.update(EcapDeriv);

  invB_ = B_.inverse();

  Y_(0) = Elat4 - ElatM;
  Y_(1) = Ecap4 - EcapM;
  K_(0) = 0 * ElatDeriv;
  K_(1) = 0 * EcapDeriv;

  X_ = invB_ * (G_ * Y_ - (A_ - K_));

  // Mise sous forme beta
  betaR = X_(0);
  betaF = X_(1);

  // Forme non linéaire de l'observateur, a activer si besoin
  /*betaR    = std::asin( (G(0,0)*Y(0)/vitesse)) - (Ecap4 +d_AR);
  betaF=std::atan(( wheelbase_/(vitesse*std::cos(betaR+d_AR)) )*(G(1,1)*Y(1)+ vitesse*courb*std::cos(Ecap4+betaR+d_AR)/(1-courb*Elat4)) +std::tan(betaR+d_AR))
      - (delta);*/

  if (counter_ < 5) {
    betaR = 0;
    betaF = 0;
  }


  if (betaR > 40 * M_PI / 180) {
    betaR = 40 * M_PI / 180;
  }
  if (betaR < -40 * M_PI / 180) {
    betaR = -40 * M_PI / 180;
  }
  if (betaF > 40 * M_PI / 180) {
    betaF = 40 * M_PI / 180;
  }
  if (betaF < -40 * M_PI / 180) {
    betaF = -40 * M_PI / 180;
  }

  // Filtrage derive cinematique
  betaRF = betaR_f_.update(betaR);
  betaFF = betaF_f_.update(betaF);

  return true;
}

//-----------------------------------------------------------------------------
bool SlidingObserverCinematicLinearTangent::computeSliding2_(
  const double & ElatM,
  const double & EcapM,
  const double & vitesse,
  const double & delta,
  const double & courb,
  const double & /*d_AR*/)
{
  const double & wheelbase_ = wheelBase_;
  const double & sampling_period_ = samplingPeriod_;


  //  Eigen::Matrix2d G,B;
  //  Eigen::Vector2d Y,X;

  //  G(0,0) = -4;
  //  G(0,1) = 0;
  //  G(1,0) = 0;
  //  G(1,1) = -2;
  //  G(0,0) = -1.2;  // Gain oservateur
  //  G(0,1) = 0;
  //  G(1,0) = 0;
  //  G(1,1) = -0.8;

  B_(0, 0) = 0;
  B_(1, 0) = vitesse * std::cos(EcapM + betaR);
  B_(0, 1) = vitesse * std::cos(betaR) * (1 + std::tan(delta + betaF) * std::tan(delta + betaF)) /
    wheelbase_;
  B_(1, 1) = vitesse * (courb * std::sin(EcapM + betaR) / (1 - courb * ElatM)) -
    vitesse * std::sin(betaR) * (std::tan(delta + betaF) - std::tan(betaR)) / wheelbase_ -
    vitesse * std::cos(betaR) * (1 + std::tan(betaR) * std::tan(betaR)) / wheelbase_;

  // B(2,1) = vitesse*std::cos(Ecap4+d_AR);
  // B(1,1) = 0;
  // B(2, 2) = vitesse * (courb * std::sin(Ecap4 + d_AR) / (1 - courb * Elat4)) -
  //  vitesse * (std::cos(d_AR) + std::tan(delta) * std::sin(d_AR)) / wheelbase_;
  // B(2,1) = (vitesse*std::cos(d_AR)/wheelbase_)*(1+pow(std::tan(delta),2));
  Y_(0) = Elat4 - ElatM;
  Y_(1) = Ecap4 - EcapM;

  X_ = B_ * G_ * Y_;

  // Mise sous forme beta
  betaF += sampling_period_ * X_(0);
  betaR += sampling_period_ * X_(1);
  if (counter_ < 2) {
    betaR = 0;
    betaF = 0;
  }
  //  static romea::FirstOrderButterworth betaR_2_f(0.96), betaF_2_f(0.96);
  betaRF = betaR_f_.update(betaF);
  betaFF = betaF_f_.update(betaR);

  return true;
}

//-----------------------------------------------------------------------------
void SlidingObserverCinematicLinearTangent::evolution_(
  const double & ElatM,
  const double & EcapM,
  const double & vitesse,
  const double & delta,
  const double & courb,
  const double & d_AR)
{
  const double & sampling_period_ = samplingPeriod_;
  const double & wheelbase_ = wheelBase_;

  Ecap4 += (vitesse * sampling_period_) *
    (std::cos(betaR + d_AR) * (std::tan(delta + betaF) - std::tan(betaR + d_AR)) / wheelbase_ -
    (courb * std::cos(Ecap4 + betaR + d_AR)) / (1 - courb * Elat4));
  Elat4 += vitesse * sampling_period_ * std::sin(Ecap4 + betaR + d_AR);
  if (counter_ < 10) {
    Elat4 = ElatM;
    Ecap4 = EcapM;
  }
  counter_++;
}

//-----------------------------------------------------------------------------
void SlidingObserverCinematicLinearTangent::evolution2_(
  const double & ElatM,
  const double & EcapM,
  const double & vitesse,
  const double & delta,
  const double & courb,
  const double & d_AR)
{
  const double & sampling_period_ = samplingPeriod_;
  const double & wheelbase_ = wheelBase_;

  double K11 = 10;
  double K12 = 10;
  Ecap4 += vitesse * sampling_period_ *
    (std::cos(betaR + d_AR) * (std::tan(delta + betaF) - std::tan(betaR + d_AR)) / wheelbase_ -
    (courb * std::cos(EcapM + betaR + d_AR)) / (1 - courb * ElatM) ) -
    K12 * sampling_period_ * (Ecap4 - EcapM);
  Elat4 += vitesse * sampling_period_ * std::sin(EcapM + betaR + d_AR) -
    K11 * sampling_period_ * (Elat4 - ElatM);

  if (counter_ < 10) {
    Elat4 = ElatM;
    Ecap4 = EcapM;
  }
  counter_++;
}

//-----------------------------------------------------------------------------
const double & SlidingObserverCinematicLinearTangent::getLateralDeviation()const
{
  return Elat4;
}

//-----------------------------------------------------------------------------
const double & SlidingObserverCinematicLinearTangent::getCourseDeviation() const
{
  return Ecap4;
}

}  // namespace core
}  // namespace romea
