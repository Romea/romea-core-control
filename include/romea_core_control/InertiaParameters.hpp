// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_CONTROL__INERTIAPARAMETERS_HPP_
#define ROMEA_CORE_CONTROL__INERTIAPARAMETERS_HPP_

#include <Eigen/Core>

namespace romea
{

struct InertiaParameters
{
  double mass;
  Eigen::Vector3d massCenter;
  double zInertialMoment;
};

}  // namespace romea

#endif  // ROMEA_CORE_CONTROL__INERTIAPARAMETERS_HPP_
