#include "test_follow_trajectory_base.hpp"
#include "romea_control/command/FollowTrajectoryClassicSliding.hpp"
#include "romea_control/observer/SlidingObserverCinematicLinearTangent.hpp"


//classic:
//  gains:
//    front_kp: 0.7
//    rear_kp: 0.5

//cinematic linear_tangent
//gains:
//  lateral_deviation: -4
//  course_deviation: -2
//filter_weights:
//  lateral_deviation: 0.9
//  course_deviation: 0.9
//  front_sliding_angle: 0.98
//  rear_sliding_angle: 0.96

class TestFollowTrajectoryClassicSliding : public TestFollowTrajectoryBase
{
public :

  TestFollowTrajectoryClassicSliding():
    TestFollowTrajectoryBase(),
    steering_angles(),
    ouput_steering_angles(),
    command(nullptr),
    observer(nullptr)
  {}

  virtual ~TestFollowTrajectoryClassicSliding()=default;

  virtual void SetUp() override
  {
    using Command = romea::FollowTrajectoryClassicSliding;
    using Observer = romea::SlidingObserverCinematicLinearTangent;
    command=std::make_unique<Command>(1.6, Command::Parameters{0.7,0.5});
    observer=std::make_unique<Observer>(0.1,1.6, Observer::Parameters{-4,-2,0.9,0.9,0.98,0.96});
  }

  virtual void readOutputData() override
  {
    output_data>>ouput_steering_angles.front;
    output_data>>ouput_steering_angles.rear;
  }

  virtual void writeEstimatedData() override
  {
    output_data<<std::setprecision(10)<<steering_angles.front<<" ";
    output_data<<std::setprecision(10)<<steering_angles.rear;
    output_data<<std::endl;
  }

  virtual void checkData() override
  {
    EXPECT_NEAR(steering_angles.front,ouput_steering_angles.front,0.0001);
    EXPECT_NEAR(steering_angles.rear,ouput_steering_angles.rear,0.0001);
  }

  virtual void update() override
  {
    observer->update(input_lateral_deviation,
                     input_course_deviation,
                     input_curvature,
                     input_linear_speed,
                     input_front_steering,
                     input_rear_steering);

    steering_angles =
        command->computeSteeringAngles(input_lateral_deviation,
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

  romea::FrontRearData steering_angles;
  romea::FrontRearData ouput_steering_angles;
  std::unique_ptr<romea::FollowTrajectoryClassicSliding> command;
  std::unique_ptr<romea::SlidingObserverCinematicLinearTangent> observer;

};

//-----------------------------------------------------------------------------
TEST_F(TestFollowTrajectoryClassicSliding, test1ws)
{
  //generateEstimatedDataFile("input_1ws.txt","output_classic_sliding_1ws.txt");
  openFiles("input_1ws.txt","output_classic_sliding_1ws.txt");
  check();
}



//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
