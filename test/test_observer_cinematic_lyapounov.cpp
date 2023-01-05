// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <memory>

// local
#include "test_follow_trajectory_base.hpp"
#include "romea_core_control/observer/SlidingObserverCinematicLyapounov.hpp"

// cinematic_lyapounov:
//  gains:
//    x_deviation: -3.
//    y_deviation: -3.
//    course_deviation: -3.
//    front_sliding_angle: -1.8
//    rear_sliding_angle: -1.22

class TestObserverCinematicLyapounov : public TestFollowTrajectoryBase
{
public:
  TestObserverCinematicLyapounov()
  : TestFollowTrajectoryBase(),
    output_x(),
    output_y(),
    output_course(),
    observer(nullptr)
  {}

  virtual ~TestObserverCinematicLyapounov() = default;

  void SetUp()override
  {
    using Observer = romea::SlidingObserverCinematicLyapounov;
    observer = std::make_unique<Observer>(
      0.1, 1.6, Observer::Parameters{-3., -3., -3., -1.8, -1.22});
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
  std::unique_ptr<romea::SlidingObserverCinematicLyapounov> observer;
};

//-----------------------------------------------------------------------------
TEST_F(TestObserverCinematicLyapounov, test1ws)
{
  // generateEstimatedDataFile("input_1ws.txt","output_cinematic_lyapounov_1ws.txt");
  openFiles("input_1ws.txt", "output_cinematic_lyapounov_1ws.txt");
  check();
}


//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
