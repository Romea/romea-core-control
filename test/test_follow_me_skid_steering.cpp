//include
#include "test_follow_me_base.hpp"
#include "romea_core_control/command/FollowMe.hpp"

//<param name="follow_me/gains/kp" value="-0.25" type="double"/>
//<param name="follow_me/gains/ki" value="0" type="double"/>
//<param name="follow_me/gains/kd" value="0.8" type="double"/>
//<param name="follow_me/gains/kdd" value="3." type="double"/>


class TestFollowMeSkidSteering : public TestFollowMeBase
{
public :

  TestFollowMeSkidSteering():
    TestFollowMeBase(),
    angular_speed_command(),
    heading_omega_d(),
    theta_error(),
    output_angular_speed_command(),
    output_heading_omega_d(),
    output_theta_error()
  {

  }

  virtual ~TestFollowMeSkidSteering()=default;

  virtual void readOutputData() override
  {
    output_data>>output_linear_speed_command;
    output_data>>output_linear_speed_omega_d;
    output_data>>output_angular_speed_command;
    output_data>>output_heading_omega_d;
    output_data>>output_theta_error;
  }

  virtual void writeEstimatedData() override
  {
    output_data<<std::setprecision(10)<<linear_speed_command<<" ";
    output_data<<std::setprecision(10)<<linear_speed_omega_d<<" ";
    output_data<<std::setprecision(10)<<angular_speed_command<<" ";
    output_data<<std::setprecision(10)<<heading_omega_d;
    output_data<<std::setprecision(10)<<theta_error;
    output_data<<std::endl;
  }

  virtual void checkData() override
  {
    EXPECT_NEAR(linear_speed_command,output_linear_speed_command,0.0001);
    EXPECT_NEAR(linear_speed_omega_d,output_linear_speed_omega_d,0.0001);
    EXPECT_NEAR(angular_speed_command,output_angular_speed_command,0.0001);
    EXPECT_NEAR(heading_omega_d,output_heading_omega_d,0.0001);
    EXPECT_NEAR(theta_error,output_theta_error,0.0001);
  }

  void computeAngularSpeedCommand()
  {
    angular_speed_command=
        follow_me->computeAngularSpeed(input_follower_lateral_deviation,
                                       input_follower_course_deviation,
                                       std::numeric_limits<double>::max(),
                                       heading_omega_d,
                                       theta_error);
  }

  virtual void update() override
  {
    computeLinearSpeedCommand();
    computeAngularSpeedCommand();
  }

  double angular_speed_command;
  double heading_omega_d;
  double theta_error;
  double output_angular_speed_command;
  double output_heading_omega_d;
  double output_theta_error;

};

//-----------------------------------------------------------------------------
TEST_F(TestFollowMeSkidSteering, test1ws)
{
  //generateEstimatedDataFile("input_1ws.txt","output_classic_sliding_1ws.txt");
  //openFiles("input_1ws.txt","output_classic_sliding_1ws.txt");
  //check();
}



//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
