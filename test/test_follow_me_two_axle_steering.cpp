// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#include "test_follow_me_base.hpp"
#include "romea_core_control/command/FollowMe.hpp"

//  <param name="follow_me/gains/kp" value="-0.25" type="double"/>
//  <param name="follow_me/gains/ki" value="0" type="double"/>
//  <param name="follow_me/gains/kd" value="0.8" type="double"/>
//  <param name="follow_me/gains/kdd" value="3." type="double"/>


class TestFollowMeTwoAxleSteering : public TestFollowMeBase
{
public:
  TestFollowMeTwoAxleSteering()
  : TestFollowMeBase(),
    steering_angles_command(),
    heading_omega_d(),
    theta_consigne(),
    output_steering_angles_command(),
    output_heading_omega_d(),
    output_theta_consigne(),
    wheelbase(1.2)
  {
  }

  virtual ~TestFollowMeTwoAxleSteering() = default;

  void readOutputData() override
  {
    output_data >> output_linear_speed_command;
    output_data >> output_linear_speed_omega_d;
    output_data >> output_steering_angles_command.front;
    output_data >> output_steering_angles_command.rear;
    output_data >> output_heading_omega_d;
    output_data >> output_theta_consigne;
  }

  void writeEstimatedData() override
  {
    output_data << std::setprecision(10) << linear_speed_command << " ";
    output_data << std::setprecision(10) << linear_speed_omega_d << " ";
    output_data << std::setprecision(10) << steering_angles_command.front << " ";
    output_data << std::setprecision(10) << steering_angles_command.rear << " ";
    output_data << std::setprecision(10) << heading_omega_d;
    output_data << std::setprecision(10) << theta_consigne;
    output_data << std::endl;
  }

  void checkData() override
  {
    EXPECT_NEAR(linear_speed_command, output_linear_speed_command, 0.0001);
    EXPECT_NEAR(linear_speed_omega_d, output_linear_speed_omega_d, 0.0001);
    EXPECT_NEAR(steering_angles_command.front, output_steering_angles_command.front, 0.0001);
    EXPECT_NEAR(steering_angles_command.rear, output_steering_angles_command.rear, 0.0001);
    EXPECT_NEAR(heading_omega_d, output_heading_omega_d, 0.0001);
    EXPECT_NEAR(theta_consigne, output_theta_consigne, 0.0001);
  }

  void computeSteeringAnglesCommand()
  {
    steering_angles_command =
      follow_me->computeSteeringAngles(
      wheelbase,
      -input_follower_lateral_deviation,
      -input_follower_course_deviation,
      input_follower_rear_steering_angle,
      M_PI_2,
      M_PI_2,
      heading_omega_d,
      theta_consigne,
      input_follower_linear_speed,
      input_leader_angular_speed,
      input_desired_longitudinal_gap);
  }

  void update() override
  {
    computeLinearSpeedCommand();
    computeSteeringAnglesCommand();
  }

  romea::FrontRearData steering_angles_command;
  double heading_omega_d;
  double theta_consigne;
  romea::FrontRearData output_steering_angles_command;
  double output_heading_omega_d;
  double output_theta_consigne;
  double wheelbase;
};

//-----------------------------------------------------------------------------
TEST_F(TestFollowMeTwoAxleSteering, test1ws)
{
  // generateEstimatedDataFile("input_1ws.txt","output_classic_sliding_1ws.txt");
  // openFiles("input_1ws.txt","output_classic_sliding_1ws.txt");
  // check();
}


//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
