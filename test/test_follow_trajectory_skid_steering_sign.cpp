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

#include <gtest/gtest.h>

#include <cmath>
#include <romea_core_control/command/FollowTrajectoryBackStepping.hpp>
#include <romea_core_control/command/FollowTrajectorySkidSliding.hpp>

using namespace romea::core;

namespace
{

struct TestCase
{
  double lateral_deviation;
  double course_deviation;
  double curvature;
  double expected_angular_speed;
};

void test_various_scenarios(
  const std::function<double(const TestCase &, double)> & compute_angular_speed)
{
  std::vector<TestCase> test_cases = {
    // straight line
    {0.0, 0.0, 0, 0},    // center, zero course
    {0.0, 1.0, 0, -1},   // center, positive course
    {0.0, -1.0, 0, 1},   // center, negative course
    {0.2, 0.0, 0, -1},   // left, zero course
    {0.2, 1.0, 0, -1},   // left, positive course
    {0.2, -1.0, 0, 1},   // left, negative course
    {2., -0.05, 0, -1},  // far left, small negative course
    {-0.2, 0.0, 0, 1},   // right, zero course
    {-0.2, 1.0, 0, -1},  // right, positive course
    {-0.2, -1.0, 0, 1},  // right, negative course
    {-2., 0.05, 0, 1},   // far right, small positive course

    {0.0, M_PI_2, 0, -1},   // center, perpendicular (left)
    {0.0, -M_PI_2, 0, 1},   // center, perpendicular (right)
    {2.0, M_PI_2, 0, -1},   // far left, perpendicular (left)
    {2.0, -M_PI_2, 0, 1},   // far left, perpendicular (right)
    {-2.0, M_PI_2, 0, -1},  // far right, perpendicular (left)
    {-2.0, -M_PI_2, 0, 1},  // far right, perpendicular (right)

    // circular trajectory
    {0.0, 0.0, 0.25, 1},   // center, left turn
    {0.0, -1.0, 0.25, 1},  // center, left turn, negative course
    {0.0, 0.02, 0.25, 1},  // center, left turn, small positive course
    {0.0, 1.2, 0.25, -1},  // center, left turn, highly positive course

    {0.0, 0.0, -0.25, -1},    // center, right turn
    {0.0, 1.0, -0.25, -1},    // center, right turn, positive course
    {0.0, -0.02, -0.25, -1},  // center, right turn, small negative course
    {0.0, -1.2, -0.25, 1},    // center, right turn, highly negative course
  };

  for (double linear_speed : {0.1, 0.5, 1.0, 2.0, -1.0}) {
    for (auto test_case : test_cases) {
      if (linear_speed < 0.) {
        test_case.expected_angular_speed *= -1;
      }

      double ang_speed = compute_angular_speed(test_case, linear_speed);

      // only check sign and zero value
      if (std::abs(test_case.expected_angular_speed) < 1e-4) {
        ASSERT_NEAR(ang_speed, 0, 1e-4);
      } else {
        ASSERT_EQ(std::signbit(ang_speed), std::signbit(test_case.expected_angular_speed))
          << "lateral, course, curvature, lin_speed: " << test_case.lateral_deviation << ", "
          << test_case.course_deviation << ", " << test_case.curvature << ", " << linear_speed
          << "\n"
          << "ang_speed, expected_ang_speed: " << ang_speed << ", "
          << test_case.expected_angular_speed << "\n";
      }
    }
  }
}

}  // namespace

class BacksteppingSign : public ::testing::Test
{
protected:
  FollowTrajectoryBackStepping::Parameters default_params{-0.5, -1, 0, 0, 0, M_PI};
  double default_sampling_period = 0.1;
  double default_wheelbase = 1.0;
};

TEST_F(BacksteppingSign, NoDeviation)
{
  FollowTrajectoryBackStepping cmd(default_sampling_period, default_wheelbase, default_params);
  double omega_d{};

  // straight line, no lateral dev, no course dev,
  double ang_speed = cmd.computeAngularSpeed(0., 0., 0., 1.0, M_PI_2, 0., omega_d);
  EXPECT_DOUBLE_EQ(ang_speed, 0);
  EXPECT_DOUBLE_EQ(omega_d, 0);
}

TEST_F(BacksteppingSign, DifferentScenarios)
{
  test_various_scenarios([this](const TestCase & test_case, double linear_speed) {
    FollowTrajectoryBackStepping cmd(default_sampling_period, default_wheelbase, default_params);
    double omega_d{};
    double ang_speed = cmd.computeAngularSpeed(
      test_case.lateral_deviation,
      test_case.course_deviation,
      test_case.curvature,
      linear_speed,
      M_PI_2,
      0.,
      omega_d);
    return ang_speed;
  });
}

TEST(SkidSliding, DifferentScenarios)
{
  test_various_scenarios([this](const TestCase & test_case, double linear_speed) {
    double target_course{};
    double ang_speed = computeBacksteppingSkidSteering(
      test_case.lateral_deviation,
      test_case.course_deviation,
      test_case.curvature,
      linear_speed,
      0.,      // linear speed disturbance
      0.,      // angular speed disturbance
      0.,      // sliding angle
      0.,      // desired lateral deviation
      M_PI_2,  // maximal angular speed
      M_PI_2,  // maximal target course
      -0.5,    // gain lateral kp
      -1.0,    // gain course kp
      target_course);
    return ang_speed;
  });
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
