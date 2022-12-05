#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>
#include <math.h>
#include <ros/ros.h>

#include "toulouse_rover/util.h"
#include "toulouse_rover/WheelSpeeds.h"
#include "toulouse_rover/wheel_speed_calculator.h"

// helper function to create a geometry_msg::Twist
auto createTwist = [](double linear_x, double linear_y, double linear_z, double angular_x, double angular_y,
                      double angular_z) {
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = linear_x;
  twist_msg.linear.y = linear_y;
  twist_msg.linear.z = linear_z;
  twist_msg.angular.x = angular_x;
  twist_msg.angular.y = angular_y;
  twist_msg.angular.z = angular_z;
  return twist_msg;
};

class WheelSpeedCalculatorFixture : public ::testing::Test
{
protected:
  ros::NodeHandle nh;
  toulouse_rover::WheelSpeeds wheel_speeds_msg;
  ros::Publisher cmd_vel_pub;
  ros::Subscriber wheel_speed_sub;
  bool got_msg = false;

  // Setup
  WheelSpeedCalculatorFixture()
  {
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    wheel_speed_sub = nh.subscribe("wheel_cmd_speeds", 1, &WheelSpeedCalculatorFixture::wheelSpeedCallback, this);
  }

  void setCmdVel(double linear_x, double linear_y, double linear_z, double angular_x, double angular_y,
                 double angular_z)
  {
    geometry_msgs::Twist cmd_vel_msg = createTwist(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z);
    cmd_vel_pub.publish(cmd_vel_msg);
  }

  void wheelSpeedCallback(const toulouse_rover::WheelSpeeds::ConstPtr& wheel_speeds)
  {
    got_msg = true;
    wheel_speeds_msg = *wheel_speeds;
  }
};

TEST_F(WheelSpeedCalculatorFixture, TestCalculatorForward)
{
  ros::NodeHandle nh;
  std::string wheel_namespace_str("");
  util::WheelConfigurationType wheel_config_type = util::WheelConfigurationType::DIFFERENTIAL_DRIVE;

  wheel_speed_calculator::WheelSpeedCalculator wheel_calc(nh, wheel_namespace_str, wheel_config_type);

  setCmdVel(0.5, 0, 0, 0, 0, 0);

  while (!got_msg)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  EXPECT_GT(wheel_speeds_msg.back_left_radians_per_sec, 0);
  EXPECT_GT(wheel_speeds_msg.back_right_radians_per_sec, 0);
}

TEST_F(WheelSpeedCalculatorFixture, TestCalculatorBackward)
{
  ros::NodeHandle nh;
  std::string wheel_namespace_str("");
  util::WheelConfigurationType wheel_config_type = util::WheelConfigurationType::DIFFERENTIAL_DRIVE;

  wheel_speed_calculator::WheelSpeedCalculator wheel_calc(nh, wheel_namespace_str, wheel_config_type);

  setCmdVel(-0.5, 0, 0, 0, 0, 0);

  while (!got_msg)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  EXPECT_LT(wheel_speeds_msg.back_left_radians_per_sec, 0);
  EXPECT_LT(wheel_speeds_msg.back_right_radians_per_sec, 0);
}

TEST_F(WheelSpeedCalculatorFixture, TestCalculatorLeft)
{
  ros::NodeHandle nh;
  std::string wheel_namespace_str("");
  util::WheelConfigurationType wheel_config_type = util::WheelConfigurationType::DIFFERENTIAL_DRIVE;

  wheel_speed_calculator::WheelSpeedCalculator wheel_calc(nh, wheel_namespace_str, wheel_config_type);

  setCmdVel(0, 0, 0, 0, 0, 3.141 / 4);

  while (!got_msg)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  EXPECT_LT(wheel_speeds_msg.back_left_radians_per_sec, wheel_speeds_msg.back_right_radians_per_sec);
}

TEST_F(WheelSpeedCalculatorFixture, TestCalculatorRight)
{
  ros::NodeHandle nh;
  std::string wheel_namespace_str("");
  util::WheelConfigurationType wheel_config_type = util::WheelConfigurationType::DIFFERENTIAL_DRIVE;

  wheel_speed_calculator::WheelSpeedCalculator wheel_calc(nh, wheel_namespace_str, wheel_config_type);

  setCmdVel(0, 0, 0, 0, 0, -3.141 / 4);

  while (!got_msg)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  EXPECT_LT(wheel_speeds_msg.back_right_radians_per_sec, wheel_speeds_msg.back_left_radians_per_sec);
}

TEST_F(WheelSpeedCalculatorFixture, TestCalculatorLeftExact)
{
  ros::NodeHandle nh;
  std::string wheel_namespace_str("");
  util::WheelConfigurationType wheel_config_type = util::WheelConfigurationType::DIFFERENTIAL_DRIVE;

  wheel_speed_calculator::WheelSpeedCalculator wheel_calc(nh, wheel_namespace_str, wheel_config_type);

  setCmdVel(0.1, 0, 0, 0, 0, 3.141 / 4);

  while (!got_msg && ros::ok())
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  double wheel_rotation_meters_per_sec = 0.1 + 3.141 / 4 * util::WHEEL_SEP_WIDTH / 2.0;
  double wheel_rotation_radians_per_sec = wheel_rotation_meters_per_sec / util::WHEEL_RADIUS;

  EXPECT_FLOAT_EQ(wheel_speeds_msg.back_right_radians_per_sec, wheel_rotation_radians_per_sec);

  wheel_rotation_meters_per_sec = 0.1 - 3.141 / 4 * util::WHEEL_SEP_WIDTH / 2.0;
  wheel_rotation_radians_per_sec = wheel_rotation_meters_per_sec / util::WHEEL_RADIUS;
  EXPECT_FLOAT_EQ(wheel_speeds_msg.back_left_radians_per_sec, wheel_rotation_radians_per_sec);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_wheel_speed_calculator_node");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
