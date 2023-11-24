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
#include <memory>

// local
#include "test_follow_trajectory_base.hpp"
#include "romea_core_control/command/FollowTrajectoryClassicSliding.hpp"
#include "romea_core_control/observer/SlidingObserverCinematicLinearTangent.hpp"


// classic:
//  gains:
//    front_kp: 0.7
//    rear_kp: 0.5

// cinematic linear_tangent
// gains:
//  lateral_deviation: -4
//  course_deviation: -2
// filter_weights:
//  lateral_deviation: 0.9
//  course_deviation: 0.9
//  front_sliding_angle: 0.98
//  rear_sliding_angle: 0.96

class TestFollowTrajectoryClassicSliding : public TestFollowTrajectoryBase
{
public:
  TestFollowTrajectoryClassicSliding()
  : TestFollowTrajectoryBase(),
    steering_angles(),
    ouput_steering_angles(),
    command(nullptr),
    observer(nullptr)
  {}

  virtual ~TestFollowTrajectoryClassicSliding() = default;

  void SetUp() override
  {
    using Command = romea::core::FollowTrajectoryClassicSliding;
    using Observer = romea::core::SlidingObserverCinematicLinearTangent;
    command = std::make_unique<Command>(1.6, Command::Parameters{0.7, 0.5});
    observer = std::make_unique<Observer>(
      0.1, 1.6, Observer::Parameters{-4, -2, 0.9, 0.9, 0.98, 0.96});
  }

  void readOutputData() override
  {
    output_data >> ouput_steering_angles.front;
    output_data >> ouput_steering_angles.rear;
  }

  void writeEstimatedData() override
  {
    output_data << std::setprecision(10) << steering_angles.front << " ";
    output_data << std::setprecision(10) << steering_angles.rear;
    output_data << std::endl;
  }

  void checkData() override
  {
    EXPECT_NEAR(steering_angles.front, ouput_steering_angles.front, 0.0001);
    EXPECT_NEAR(steering_angles.rear, ouput_steering_angles.rear, 0.0001);
  }

  void update() override
  {
    observer->update(
      input_lateral_deviation,
      input_course_deviation,
      input_curvature,
      input_linear_speed,
      input_front_steering,
      input_rear_steering);

    steering_angles =
      command->computeSteeringAngles(
      input_lateral_deviation,
      input_course_deviation,
      input_curvature,
      observer->getFrontSlidingAngle(),
      observer->getRearSlidingAngle(),
      input_rear_steering,
      M_PI_2,
      M_PI_2,
      0,
      0);
  }

  romea::core::FrontRearData steering_angles;
  romea::core::FrontRearData ouput_steering_angles;
  std::unique_ptr<romea::core::FollowTrajectoryClassicSliding> command;
  std::unique_ptr<romea::core::SlidingObserverCinematicLinearTangent> observer;
};

//-----------------------------------------------------------------------------
TEST_F(TestFollowTrajectoryClassicSliding, test1ws)
{
  // generateEstimatedDataFile("input_1ws.txt","output_classic_sliding_1ws.txt");
  openFiles("input_1ws.txt", "output_classic_sliding_1ws.txt");
  check();
}


//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
