#include <gtest/gtest.h>
#include <ros/console.h>
#include <ros/ros.h>

#include "toulouse_rover/util.h"
#include "toulouse_rover/WheelPwmSpeeds.h"
#include "toulouse_rover/WheelSpeeds.h"
#include "toulouse_rover/wheel_speed_controller.h"

// helper function to create a toulouse_rover::WheelSpeed
auto createWheelSpeedMsg = [](double front_left, double front_right, double back_left, double back_right) {
  toulouse_rover::WheelSpeeds wheel_speed_msg;
  wheel_speed_msg.front_left_radians_per_sec = front_left;
  wheel_speed_msg.front_right_radians_per_sec = front_right;
  wheel_speed_msg.back_left_radians_per_sec = back_left;
  wheel_speed_msg.back_right_radians_per_sec = back_right;
  return wheel_speed_msg;
};

class WheelSpeedControllerFixture : public ::testing::Test
{
protected:
  ros::NodeHandle nh;
  toulouse_rover::WheelPwmSpeeds wheel_speed_msg;
  ros::Publisher wheel_speed_pub;
  ros::Subscriber pwm_sub;
  bool got_msg = false;

  // Setup
  WheelSpeedControllerFixture()
  {
    wheel_speed_pub = nh.advertise<toulouse_rover::WheelSpeeds>("/wheels/wheel_cmd_speeds", 1);
    pwm_sub = nh.subscribe("/wheels/wheels_pwm_cmd", 10, &WheelSpeedControllerFixture::wheelPwmSpeedMsgCB, this);
  }

  void setWheelSpeeds(double front_left, double front_right, double back_left, double back_right)
  {
    toulouse_rover::WheelSpeeds wheel_speeds_msg = createWheelSpeedMsg(front_left, front_right, back_left, back_right);
    wheel_speed_pub.publish(wheel_speeds_msg);
  }

  void wheelPwmSpeedMsgCB(const toulouse_rover::WheelPwmSpeeds::ConstPtr& wheel_pwm_msg)
  {
    got_msg = true;
    wheel_speed_msg = *wheel_pwm_msg;
  }

  toulouse_rover::WheelPwmSpeeds getReceivedMsg()
  {
    got_msg = false;
    return wheel_speed_msg;
  }
};

TEST_F(WheelSpeedControllerFixture, TestControllerForward)
{
  ros::NodeHandle nh;
  std::string wheel_namespace_str("wheels");
  util::WheelConfigurationType wheel_config_type = util::WheelConfigurationType::DIFFERENTIAL_DRIVE;
  float loop_rate = 20;
  bool use_pid = false;

  ros::Duration(0.5).sleep();

  wheel_speed_controller::WheelSpeedController wheel_controller(nh, wheel_namespace_str, use_pid, wheel_config_type,
                                                                loop_rate);

  setWheelSpeeds(0, 0, 3.141 / 4, 3.141 / 4);
  // give the subscribers in WheelSpeedController time to get message
  for (int i = 0; i < 5; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  // first message is initial enable motors message with 0 speeds
  wheel_controller.spinOnce();
  while (ros::ok() && !got_msg)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  toulouse_rover::WheelPwmSpeeds pwm_speeds = getReceivedMsg();

  while (ros::ok() && !got_msg)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  pwm_speeds = getReceivedMsg();

  // back wheels
  EXPECT_FLOAT_EQ(pwm_speeds.back_right_pwm_2, 0);
  EXPECT_GT(pwm_speeds.back_right_pwm_1, 0);

  EXPECT_FLOAT_EQ(pwm_speeds.back_left_pwm_2, 0);
  EXPECT_GT(pwm_speeds.back_left_pwm_1, 0);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_wheel_speed_controller_node");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
