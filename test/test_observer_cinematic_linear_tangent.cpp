#include "test_follow_trajectory_base.hpp"
#include "romea_core_control/observer/SlidingObserverCinematicLinearTangent.hpp"


//cinematic_linear_tangent:
//  gains:
//    lateral_deviation: -4
//    course_deviation: -2
//  filter_weights:
//    lateral_deviation: 0.9
//    course_deviation: 0.9
//    front_sliding_angle: 0.98
//    rear_sliding_angle: 0.96

class TestObserverCinematicLinearTangent : public TestFollowTrajectoryBase
{
public :

  TestObserverCinematicLinearTangent():
    TestFollowTrajectoryBase(),
    output_lateral_deviation(),
    output_course_deviation(),
    observer(nullptr)
  {}

  virtual ~TestObserverCinematicLinearTangent()=default;

  virtual void SetUp()override
  {
    using Observer = romea::SlidingObserverCinematicLinearTangent;
    observer=std::make_unique<Observer>(0.1,1.6, Observer::Parameters{-4,-2,0.9,0.9,0.98,0.96});
  }

  virtual void readOutputData() override
  {
    output_data>>output_lateral_deviation;
    output_data>>output_course_deviation;
    output_data>>output_front_sliding;
    output_data>>output_rear_sliding;
  }

  virtual void writeEstimatedData() override
  {
    output_data<<std::setprecision(10)<<observer->getLateralDeviation()<<" ";
    output_data<<std::setprecision(10)<<observer->getCourseDeviation()<<" ";
    output_data<<std::setprecision(10)<<observer->getFrontSlidingAngle()<<" ";
    output_data<<std::setprecision(10)<<observer->getRearSlidingAngle();
    output_data<<std::endl;
  }

  virtual void checkData() override
  {
    EXPECT_NEAR(observer->getLateralDeviation(),output_lateral_deviation,0.0001);
    EXPECT_NEAR(observer->getCourseDeviation(),output_course_deviation,0.0001);
    EXPECT_NEAR(observer->getFrontSlidingAngle(),output_front_sliding,0.0001);
    EXPECT_NEAR(observer->getRearSlidingAngle(),output_rear_sliding,0.0001);
  }

  virtual void update() override
  {
    observer->update(input_lateral_deviation,
                     input_course_deviation,
                     input_curvature,
                     input_linear_speed,
                     input_front_steering,
                     input_rear_steering);
  }


  double output_lateral_deviation;
  double output_course_deviation;
  std::unique_ptr<romea::SlidingObserverCinematicLinearTangent> observer;
};

//-----------------------------------------------------------------------------
TEST_F(TestObserverCinematicLinearTangent, test1ws)
{
  //generateEstimatedDataFile("input_1ws.txt","output_cinematic_linear_tangent_1ws.txt");
  openFiles("input_1ws.txt","output_cinematic_linear_tangent_1ws.txt");
  check();
}



//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
