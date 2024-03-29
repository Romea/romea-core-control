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


#ifndef TEST_FOLLOW_ME_BASE_HPP_
#define TEST_FOLLOW_ME_BASE_HPP_


// std
#include <memory>
#include <limits>

// local
#include "test_follow_base.hpp"
#include "romea_core_control/command/KeepInterdistance.hpp"
#include "romea_core_control/command/FollowMe.hpp"


class TestFollowMeBase : public TestFollow
{
public:
  TestFollowMeBase()
  : input_desired_longitudinal_gap(),
    input_desired_lateral_gap(),
    input_leader_longitudinal_deviation(),
    input_leader_lateral_deviation(),
    input_leader_linear_speed(),
    input_leader_angular_speed(),
    input_follower_longitudinal_deviation(),
    input_follower_lateral_deviation(),
    input_follower_course_deviation(),
    input_follower_linear_speed(),
    input_follower_angular_speed(),
    input_follower_front_steering_angle(),
    input_follower_rear_steering_angle(),
    output_linear_speed_command(),
    output_linear_speed_omega_d(),
    linear_speed_command(),
    linear_speed_omega_d(),
    keep_interdistance(nullptr),
    follow_me(nullptr)
  {
  }

  virtual ~TestFollowMeBase() = default;

  void SetUp()override
  {
    romea::core::FollowMe::Parameters gains{-0.25, 0, 0.8, 3.};
    follow_me = std::make_unique<romea::core::FollowMe>(0.1, gains);
    keep_interdistance = std::make_unique<romea::core::KeepInterdistance>(0.1);
  }

  void readInputData() override
  {
    input_data >> input_desired_longitudinal_gap;
    input_data >> input_desired_lateral_gap;
    input_data >> input_leader_longitudinal_deviation;
    input_data >> input_leader_lateral_deviation;
    input_data >> input_leader_linear_speed;
    input_data >> input_leader_angular_speed;
    input_data >> input_follower_longitudinal_deviation;
    input_data >> input_follower_lateral_deviation;
    input_data >> input_follower_course_deviation;
    input_data >> input_follower_linear_speed;
    input_data >> input_follower_angular_speed;
    input_data >> input_follower_front_steering_angle;
    input_data >> input_follower_rear_steering_angle;
  }

  void computeLinearSpeedCommand()
  {
    linear_speed_command =
      keep_interdistance->computeFollowerSpeed(
      input_desired_longitudinal_gap,
      input_desired_lateral_gap,
      input_leader_longitudinal_deviation,
      input_follower_longitudinal_deviation,
      input_leader_linear_speed,
      input_leader_lateral_deviation,
      input_follower_course_deviation,
      input_follower_linear_speed,
      std::numeric_limits<double>::max(),
      linear_speed_omega_d,
      input_leader_angular_speed);
  }

  double input_desired_longitudinal_gap;
  double input_desired_lateral_gap;
  double input_leader_longitudinal_deviation;
  double input_leader_lateral_deviation;
  double input_leader_linear_speed;
  double input_leader_angular_speed;
  double input_follower_longitudinal_deviation;
  double input_follower_lateral_deviation;
  double input_follower_course_deviation;
  double input_follower_linear_speed;
  double input_follower_angular_speed;
  double input_follower_front_steering_angle;
  double input_follower_rear_steering_angle;

  double output_linear_speed_command;
  double output_linear_speed_omega_d;

  double linear_speed_command;
  double linear_speed_omega_d;

  std::unique_ptr<romea::core::KeepInterdistance> keep_interdistance;
  std::unique_ptr<romea::core::FollowMe> follow_me;
};

#endif  // TEST_FOLLOW_ME_BASE_HPP_
