#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>
#include <math.h>
#include <mutex>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

#include "toulouse_rover/util.h"
#include "toulouse_rover/WheelSpeeds.h"
#include "toulouse_rover/odom_calculator.h"

// helper function to create a toulouse_rover::WheelSpeed
auto createWheelSpeedMsg = [](double front_left, double front_right, double back_left, double back_right) {
  toulouse_rover::WheelSpeeds wheel_speed_msg;
  wheel_speed_msg.front_left_radians_per_sec = front_left;
  wheel_speed_msg.front_right_radians_per_sec = front_right;
  wheel_speed_msg.back_left_radians_per_sec = back_left;
  wheel_speed_msg.back_right_radians_per_sec = back_right;
  return wheel_speed_msg;
};

class OdomCalculatorFixture : public ::testing::Test
{
protected:
  ros::NodeHandle nh;
  nav_msgs::Odometry odom_msg;
  ros::Subscriber odom_sub;
  ros::Publisher wheel_speed_pub;
  std::mutex get_msg_mutex_;
  bool got_msg = false;

  // Setup
  OdomCalculatorFixture()
  {
    wheel_speed_pub = nh.advertise<toulouse_rover::WheelSpeeds>("wheel_speeds", 1);
    odom_sub = nh.subscribe("odom", 1, &OdomCalculatorFixture::odomCallback, this);
  }

  void setWheelSpeeds(double front_left, double front_right, double back_left, double back_right)
  {
    toulouse_rover::WheelSpeeds wheel_speeds_msg = createWheelSpeedMsg(front_left, front_right, back_left, back_right);
    wheel_speed_pub.publish(wheel_speeds_msg);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
  {
    const std::lock_guard<std::mutex> lock(get_msg_mutex_);
    got_msg = true;
    odom_msg = *odom;
  }

  nav_msgs::Odometry getMsg()
  {
    const std::lock_guard<std::mutex> lock(get_msg_mutex_);
    got_msg = false;
    return odom_msg;
  }

  void wait_for_msg()
  {
    while (ros::ok())
    {
      ros::spinOnce();
      ros::Duration(0.01).sleep();
      const std::lock_guard<std::mutex> lock(get_msg_mutex_);
      if (got_msg)
      {
        break;
      }
    }
  }
};

TEST_F(OdomCalculatorFixture, TestCalculatorForward)
{
  ros::NodeHandle nh;
  util::WheelConfigurationType wheel_config_type = util::WheelConfigurationType::DIFFERENTIAL_DRIVE;

  odom_calculator::OdomCalculator odom_calc(nh, wheel_config_type);

  setWheelSpeeds(0, 0, 0, 0);

  wait_for_msg();

  nav_msgs::Odometry initialMessage = getMsg();
  ros::Duration(0.25).sleep();

  setWheelSpeeds(0, 0, 3.141 / 4, 3.141 / 4);
  wait_for_msg();

  nav_msgs::Odometry subsequent_msg = getMsg();

  float approximate_x_movement = util::convert_radians_per_sec_to_meters_per_sec(3.141 / 4) * 0.25;
  EXPECT_GT(subsequent_msg.pose.pose.position.x, 0);
  EXPECT_NEAR(subsequent_msg.pose.pose.position.x, approximate_x_movement, 0.003);
}

TEST_F(OdomCalculatorFixture, TestCalculatorTurnLeft)
{
  ros::NodeHandle nh;
  util::WheelConfigurationType wheel_config_type = util::WheelConfigurationType::DIFFERENTIAL_DRIVE;

  odom_calculator::OdomCalculator odom_calc(nh, wheel_config_type);

  setWheelSpeeds(0, 0, 0, 0);

  wait_for_msg();

  nav_msgs::Odometry initialMessage = getMsg();
  ros::Duration(0.25).sleep();

  setWheelSpeeds(0, 0, 3.141 / 8, 3.141 / 4);
  wait_for_msg();

  nav_msgs::Odometry subsequent_msg = getMsg();
  float approximate_distance_left_wheel = util::convert_radians_per_sec_to_meters_per_sec(3.141 / 8) * 0.25;
  float approximate_distance_right_wheel = util::convert_radians_per_sec_to_meters_per_sec(3.141 / 4) * 0.25;
  float approximate_distance_robot = (approximate_distance_left_wheel + approximate_distance_right_wheel) / 2;

  float approximate_angle_change =
      (approximate_distance_right_wheel - approximate_distance_left_wheel) / (util::WHEEL_SEP_WIDTH);
  float approximate_x_change = approximate_distance_robot * cos(0 + approximate_angle_change / 2);
  float approximate_y_change = approximate_distance_robot * sin(0 + approximate_angle_change / 2);
  tf2::Quaternion orientation_quaternion;
  orientation_quaternion.setRPY(0, 0, approximate_angle_change);
  EXPECT_NEAR(subsequent_msg.pose.pose.position.x, approximate_x_change, 0.003);
  EXPECT_NEAR(subsequent_msg.pose.pose.position.y, approximate_y_change, 0.003);
  EXPECT_NEAR(subsequent_msg.pose.pose.orientation.z, orientation_quaternion.getZ(), 0.003);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_odom_calculator_node");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}