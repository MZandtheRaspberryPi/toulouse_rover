#include <gtest/gtest.h>
#include <ros/console.h>
#include <ros/ros.h>

#include "toulouse_rover/util.h"
#include "toulouse_rover/WheelSpeeds.h"
#include "toulouse_rover/wheel_speed_controller.h"
#include "i2cpwm_board/ServoArray.h"

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
  i2cpwm_board::ServoArray servo_arr_msg;
  ros::Publisher wheel_speed_pub;
  ros::Subscriber servo_array_sub;
  bool got_msg = false;

  // Setup
  WheelSpeedControllerFixture()
  {
    wheel_speed_pub = nh.advertise<toulouse_rover::WheelSpeeds>("wheel_cmd_speeds", 1);
    servo_array_sub = nh.subscribe("servos_absolute", 10, &WheelSpeedControllerFixture::servoArrCallback, this);
  }

  void setWheelSpeeds(double front_left, double front_right, double back_left, double back_right)
  {
    toulouse_rover::WheelSpeeds wheel_speeds_msg = createWheelSpeedMsg(front_left, front_right, back_left, back_right);
    wheel_speed_pub.publish(wheel_speeds_msg);
  }

  void servoArrCallback(const i2cpwm_board::ServoArray::ConstPtr& servo_arr)
  {
    got_msg = true;
    servo_arr_msg = *servo_arr;
  }

  i2cpwm_board::ServoArray getReceivedMsg()
  {
    got_msg = false;
    return servo_arr_msg;
  }
};

TEST_F(WheelSpeedControllerFixture, TestControllerPIDForward)
{
  ros::NodeHandle nh;
  std::string wheel_namespace_str("");
  ROS_WARN("PID Forward Test");
  util::WheelConfigurationType wheel_config_type = util::WheelConfigurationType::DIFFERENTIAL_DRIVE;
  float loop_rate = 50;
  bool use_pid = true;

  ROS_WARN("initting object");
  wheel_speed_controller::WheelSpeedController wheel_controller(nh, wheel_namespace_str, use_pid, wheel_config_type,
                                                                loop_rate);
  // ros::Duration(0.5).sleep();

  ROS_WARN("setting wheel speeds");
  setWheelSpeeds(0, 0, 3.141 / 4, 3.141 / 4);
  ROS_WARN("done setting wheel speeds");
  // give the subscribers in WheelSpeedController time to get message
  for (int i = 0; i < 5; i++)
  {
    ROS_WARN("waiting");
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  ROS_WARN("spinnign wheel speed controller");
  wheel_controller.spinOnce();
  ROS_WARN("done spinning wheel speed controller");
  while (!got_msg && ros::ok())
  {
    ROS_WARN("waiting for msg");
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  i2cpwm_board::ServoArray servo_speeds = getReceivedMsg();

  // back wheels
  EXPECT_FLOAT_EQ(servo_speeds.servos[8].value, 0);
  EXPECT_GT(servo_speeds.servos[9].value, 0);

  EXPECT_FLOAT_EQ(servo_speeds.servos[11].value, 0);
  EXPECT_GT(servo_speeds.servos[10].value, 0);

  // front wheels servo index 12, 13, 14, 15
  for (int i = 12; i < 16; i++)
  {
    EXPECT_FLOAT_EQ(servo_speeds.servos[i].value, 0);
  }
}

/*

TEST_F(WheelSpeedControllerFixture, TestControllerPIDLeft)
{
  ros::NodeHandle nh;
  std::string wheel_namespace_str("");
  ROS_WARN("PID Left Test");
  util::WheelConfigurationType wheel_config_type = util::WheelConfigurationType::DIFFERENTIAL_DRIVE;
  float loop_rate = 50;
  bool use_pid = true;

  wheel_speed_controller::WheelSpeedController wheel_controller(nh, wheel_namespace_str, use_pid, wheel_config_type,
                                                                loop_rate);
  ros::Duration(0.1).sleep();

  setWheelSpeeds(0, 0, 3.141 / 8, 3.141 / 4);
  // give the subscribers in WheelSpeedController time to get message
  for (int i = 0; i < 5; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  wheel_controller.spinOnce();
  while (!got_msg && ros::ok())
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  i2cpwm_board::ServoArray servo_speeds = getReceivedMsg();

  // back wheels
  EXPECT_FLOAT_EQ(servo_speeds.servos[8].value, 0);
  EXPECT_GT(servo_speeds.servos[9].value, 0);

  EXPECT_FLOAT_EQ(servo_speeds.servos[11].value, 0);
  EXPECT_GT(servo_speeds.servos[10].value, 0);

  EXPECT_GT(servo_speeds.servos[10].value, servo_speeds.servos[9].value);

  // front wheels servo index 12, 13, 14, 15
  for (int i = 12; i < 16; i++)
  {
    EXPECT_FLOAT_EQ(servo_speeds.servos[i].value, 0);
  }
}
*/

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_wheel_speed_controller_pid_node");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
