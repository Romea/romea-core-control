#ifndef _romea_InertialParameters_hpp_
#define _romea_InertialParameters_hpp_

#include <Eigen/Core>

namespace romea
{

  struct InertiaParameters
  {
    double mass;
    Eigen::Vector3d massCenter;
    double zInertialMoment;
  };
}

#endif
