#include "test_follow_base.hpp"
#include "romea_control/command/KeepInterdistance.hpp"
#include "romea_control/command/FollowMe.hpp"


class TestFollowMeBase : public TestFollow
{

public :

  TestFollowMeBase():
    input_desired_longitudinal_gap(),
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

  virtual ~TestFollowMeBase()=default;

  virtual void SetUp()override
  {
    romea::FollowMe::Parameters gains{-0.25,0,0.8,3.};
    follow_me=std::make_unique<romea::FollowMe>(0.1,gains);
    keep_interdistance=std::make_unique<romea::KeepInterdistance>(0.1);
  }

  void readInputData() override
  {
    input_data>>input_desired_longitudinal_gap;
    input_data>>input_desired_lateral_gap;
    input_data>>input_leader_longitudinal_deviation;
    input_data>>input_leader_lateral_deviation;
    input_data>>input_leader_linear_speed;
    input_data>>input_leader_angular_speed;
    input_data>>input_follower_longitudinal_deviation;
    input_data>>input_follower_lateral_deviation;
    input_data>>input_follower_course_deviation;
    input_data>>input_follower_linear_speed;
    input_data>>input_follower_angular_speed;
    input_data>>input_follower_front_steering_angle;
    input_data>>input_follower_rear_steering_angle;
  }

  void computeLinearSpeedCommand()
  {
    linear_speed_command=
        keep_interdistance->computeFollowerSpeed(input_desired_longitudinal_gap,
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

  std::unique_ptr<romea::KeepInterdistance> keep_interdistance;
  std::unique_ptr<romea::FollowMe> follow_me;
};


