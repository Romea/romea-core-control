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

#ifndef ROMEA_CORE_CONTROL__OBSERVER__SPEEDOBSERVER_HPP_
#define ROMEA_CORE_CONTROL__OBSERVER__SPEEDOBSERVER_HPP_

// std
#include <iostream>

// romea
#include "romea_core_control/SpeedAngleData.hpp"

namespace romea::core
{

class SpeedObserver
{
public:
  explicit SpeedObserver();

  virtual ~SpeedObserver() = default;

public:
  SpeedAngleData getSpeedAngle() const;

  [[nodiscard]] virtual double getSpeed() const = 0;
  [[nodiscard]] virtual double getAngle() const = 0;

  virtual void reset();

protected:
  bool is_initialized_;
};

std::ostream & operator<<(std::ostream & os, const SpeedObserver & observer);

}  // namespace romea::core

#endif  // ROMEA_CORE_CONTROL__OBSERVER__SPEEDOBSERVER_HPP_
