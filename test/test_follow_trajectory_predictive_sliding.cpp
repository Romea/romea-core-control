#include "test_follow_trajectory_base.hpp"
#include "romea_control/command/FollowTrajectoryPredictiveSliding.hpp"
#include "romea_control/observer/SlidingObserverCinematicLyapounov.hpp"


//predictive:
//  gains:
//    front_kp: 0.7
//    rear_kp: 0.7
//  prediction:
//    horizon: 10
//    a0: 0.1642
//    a1: 0.1072
//    b1: 1.0086
//    b2: -0.2801

//cinematic_lyapounov:
//  gains:
//    x_deviation: -3.
//    y_deviation: -3.
//    course_deviation: -3.
//    front_sliding_angle: -1.8
//    rear_sliding_angle: -1.22

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
    using Command = romea::FollowTrajectoryPredictiveSliding;
    using Observer = romea::SlidingObserverCinematicLyapounov;
    command=std::make_unique<Command>(1.6, Command::Parameters{0.7,0.7,10,0.1642,0.1072,1.0086,-0.2801});
    observer=std::make_unique<Observer>(0.1,1.6, Observer::Parameters{-3.,-3.,-3.,-1.8,-1.22});
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
    observer->update(input_x,
                     input_y,
                     input_course,
                     input_linear_speed,
                     input_front_steering,
                     input_rear_steering);

    steering_angles =
        command->computeSteeringAngles(input_lateral_deviation,
                                       input_course_deviation,
                                       input_curvature,
                                       input_future_curvature,
                                       input_front_steering,
                                       input_rear_steering,
                                       observer->getFrontSlidingAngle(),
                                       observer->getRearSlidingAngle(),
                                       M_PI_2,
                                       M_PI_2,
                                       0,
                                       0,
                                       0);

  }

  romea::FrontRearData steering_angles;
  romea::FrontRearData ouput_steering_angles;
  std::unique_ptr<romea::FollowTrajectoryPredictiveSliding> command;
  std::unique_ptr<romea::SlidingObserverCinematicLyapounov> observer;

};

//-----------------------------------------------------------------------------
TEST_F(TestFollowTrajectoryClassicSliding, test1ws)
{
  //  generateEstimatedDataFile("input_1ws.txt","output_predictive_sliding_1ws.txt");
  openFiles("input_1ws.txt","output_predictive_sliding_1ws.txt");
  check();
}



//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
