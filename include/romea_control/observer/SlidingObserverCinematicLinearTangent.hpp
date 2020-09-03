#ifndef _romea_SlidingObserverCinematicLinearTangent_hpp_
#define _romea_SlidingObserverCinematicLinearTangent_hpp_

//romea
#include "SlidingObserverCinematic.hpp"
#include <romea_common/signal/FirstOrderButterworth.hpp>

//eigen
#include <Eigen/Dense>

namespace romea {


class SlidingObserverCinematicLinearTangent : public SlidingObserverCinematic
{

public :

  struct Parameters
  {
    double lateralDeviationGain;
    double courseDeviationGain;
    double lateralDeviationFilterWeight;
    double courseDeviationFilterWeight;
    double frontSlidingAngleFilterWeight;
    double rearSlidingAngleFilterWeight;
  };


public:

  SlidingObserverCinematicLinearTangent(const double & samplingPeriod,
                                        const double & wheelBase,
                                        const Parameters & parameters);

  virtual ~SlidingObserverCinematicLinearTangent()=default;


public :

  void update(const double & lateralDeviation,
              const double & courseDeviation,
              const double & curvature,
              const double & linearSpeed,
              const double & frontSteeringAngle,
              const double & rearSteeringAngle);

  virtual const double & getFrontSlidingAngle() const override;

  virtual const double & getRearSlidingAngle() const override;

  const double & getLateralDeviation()const;

  const double & getCourseDeviation()const;


private :

  void initObserver_(const double& ElatM,
                     const double& EcapM);


  /**
   * Compute sliding
   * @param[in] lateral_deviation : lateral deviation in meters, positive on the right
   * @param[in] cap_deviation : cap deviation in radian, counterclockwise
   * @param[in] speed : linear speed of the robot
   * @param[in] front_steering : in radian, counterclockwise
   * @param[in] curvature at the point of the trajectory which is nearest to the robot
   * @param[in] rear_steering : in radian, counterclockwise
  **/
  bool computeSliding_(const double& lateral_deviation,
                       const double& cap_deviation,
                       const double& speed,
                       const double& front_steering,
                       const double& curvature,
                       const double& rear_steering);

  bool computeSliding2_(const double& lateral_deviation,
                        const double& cap_deviation,
                        const double& speed,
                        const double& front_steering,
                        const double& curvature,
                        const double& rear_steering);

  /**
   * evolution of sliding observer
   * @param[in] lateral_deviation : lateral deviation in meters, positive on the right
   * @param[in] cap_deviation : cap deviation in radian, counterclockwise
   * @param[in] speed : linear speed of the robot
   * @param[in] front_steering : in radian, counterclockwise
   * @param[in] curvature at the point of the trajectory which is nearest to the robot
   * @param[in] rear_steering : in radian, counterclockwise
  **/
  void evolution_(const double&  lateral_deviation,
                  const double&  cap_deviation,
                  const double&  speed,
                  const double& front_steering,
                  const double& curvature,
                  const double& rear_steering);

  void evolution2_(const double&  lateral_deviation,
                   const double&  cap_deviation,
                   const double&  speed,
                   const double& front_steering,
                   const double& curvature,
                   const double& rear_steering);


private :

  double wheelBase_;

  double betaR;
  double betaF;
  double betaRF;
  double betaFF;
  double Elat4;
  double Ecap4;
  double Elat_av;
  double Ecap_av;
  double ElatDeriv;
  double EcapDeriv;
  unsigned int counter_;

  Eigen::Matrix2d G_,B_,invB_;
  Eigen::Vector2d A_,Y_,K_,X_;
  FirstOrderButterworth lateral_deviation_drift_f_;
  FirstOrderButterworth cap_deviation_drift_f_;
  FirstOrderButterworth betaR_f_;
  FirstOrderButterworth betaF_f_;

};

}

#endif
