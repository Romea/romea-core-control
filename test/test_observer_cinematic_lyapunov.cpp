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
#include <iomanip>
#include <memory>

// local
#include "test_follow_trajectory_base.hpp"
#include "romea_core_control/observer/SlidingObserverCinematicLyapunov.hpp"

// cinematic_lyapunov:
//  gains:
//    x_deviation: -3.
//    y_deviation: -3.
//    course_deviation: -3.
//    front_sliding_angle: -1.8
//    rear_sliding_angle: -1.22

class TestObserverCinematicLyapunov : public TestFollowTrajectoryBase
{
public:
  TestObserverCinematicLyapunov()
  : TestFollowTrajectoryBase(),
    output_x(),
    output_y(),
    output_course(),
    observer(nullptr)
  {}

  virtual ~TestObserverCinematicLyapunov() = default;

  void SetUp()override
  {
    using Observer = romea::core::SlidingObserverCinematicLyapunov;
    observer = std::make_unique<Observer>(
      1.6, Observer::Parameters{-3., -3., -3., -1.8, -1.22});
  }

  void readOutputData() override
  {
    output_data >> output_x;
    output_data >> output_y;
    output_data >> output_course;
    output_data >> output_front_sliding;
    output_data >> output_rear_sliding;
  }

  void writeEstimatedData() override
  {
    output_data << std::setprecision(10) << observer->getX() << " ";
    output_data << std::setprecision(10) << observer->getY() << " ";
    output_data << std::setprecision(10) << observer->getTheta() << " ";
    output_data << std::setprecision(10) << observer->getFrontSlidingAngle() << " ";
    output_data << std::setprecision(10) << observer->getRearSlidingAngle();
    output_data << std::endl;
  }

  void checkData() override
  {
    EXPECT_NEAR(observer->getX(), output_x, 0.0001);
    EXPECT_NEAR(observer->getY(), output_y, 0.0001);
    EXPECT_NEAR(observer->getTheta(), output_course, 0.0001);
    EXPECT_NEAR(observer->getFrontSlidingAngle(), output_front_sliding, 0.0001);
    EXPECT_NEAR(observer->getRearSlidingAngle(), output_rear_sliding, 0.0001);
  }

  void update() override
  {
    observer->update(
      0.1,
      input_x,
      input_y,
      input_course,
      input_linear_speed,
      input_front_steering,
      input_rear_steering);
  }


  double output_x;
  double output_y;
  double output_course;
  std::unique_ptr<romea::core::SlidingObserverCinematicLyapunov> observer;
};

//-----------------------------------------------------------------------------
TEST_F(TestObserverCinematicLyapunov, test1ws)
{
  // generateEstimatedDataFile("input_1ws.txt","output_cinematic_lyapunov_1ws.txt");
  openFiles("input_1ws.txt", "output_cinematic_lyapunov_1ws.txt");
  check();
}


//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
