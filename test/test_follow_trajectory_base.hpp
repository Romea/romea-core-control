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


#ifndef TEST_FOLLOW_TRAJECTORY_BASE_HPP_
#define TEST_FOLLOW_TRAJECTORY_BASE_HPP_


#include "test_follow_base.hpp"

class TestFollowTrajectoryBase : public TestFollow
{
public:
  TestFollowTrajectoryBase()
  : input_x(),
    input_y(),
    input_course(),
    input_lateral_deviation(),
    input_course_deviation(),
    input_curvature(),
    input_future_curvature(),
    input_linear_speed(),
    input_front_steering(),
    input_rear_steering(),
    output_front_sliding(),
    output_rear_sliding()
  {
  }

  virtual ~TestFollowTrajectoryBase() = default;

  void readInputData() override
  {
    input_data >> input_x;
    input_data >> input_y;
    input_data >> input_course;
    input_data >> input_lateral_deviation;
    input_data >> input_course_deviation;
    input_data >> input_curvature;
    input_data >> input_future_curvature;
    input_data >> input_linear_speed;
    input_data >> input_front_steering;
    input_data >> input_rear_steering;
  }

  double input_x;
  double input_y;
  double input_course;
  double input_lateral_deviation;
  double input_course_deviation;
  double input_curvature;
  double input_future_curvature;
  double input_linear_speed;
  double input_front_steering;
  double input_rear_steering;
  double output_front_sliding;
  double output_rear_sliding;
};

#endif  // TEST_FOLLOW_TRAJECTORY_BASE_HPP_
