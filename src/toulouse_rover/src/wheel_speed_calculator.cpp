#include "toulouse_rover/wheel_speed_calculator.h"

#include <geometry_msgs/Twist.h>
#include <math.h>
#include <ros/ros.h>

#include <cmath>

#include "toulouse_rover/WheelSpeeds.h"
#include "toulouse_rover/util.h"

namespace wheel_speed_calculator
{
BaseWheelSpeedCalculator::BaseWheelSpeedCalculator(util::WheelConfigurationType wheel_config_type)
  : wheel_config_type_(wheel_config_type)
{
}

WheelSpeedCalculator::WheelSpeedCalculator(ros::NodeHandle& nh, const std::string wheel_namespace,
                                           util::WheelConfigurationType wheel_config_type)
  : wheel_namespace_(wheel_namespace)
  , wheel_config_type_(wheel_config_type)
  , front_left_wheel_(wheel_config_type)
  , front_right_wheel_(wheel_config_type)
  , back_right_wheel_(wheel_config_type)
  , back_left_wheel_(wheel_config_type)
{
  setupPubsSubs(nh, wheel_namespace);

  wheel_speeds_msg_.front_left_radians_per_sec = 0.0;
  wheel_speeds_msg_.front_right_radians_per_sec = 0.0;
  wheel_speeds_msg_.back_left_radians_per_sec = 0.0;
  wheel_speeds_msg_.back_right_radians_per_sec = 0.0;
}

void WheelSpeedCalculator::velocityCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
  const std::lock_guard<std::mutex> lock(vel_update_mtx_);
  latest_vel_msg_ = *vel;
  calcAndPublishWheelSpeeds(latest_vel_msg_);
}

void WheelSpeedCalculator::setupPubsSubs(ros::NodeHandle& nh, const std::string wheel_namespace)
{
  wheel_speeds_pub_ = nh.advertise<toulouse_rover::WheelSpeeds>("/" + wheel_namespace + "/wheel_cmd_speeds", 1);
  ctrl_sub_ = nh.subscribe("cmd_vel", 1, &WheelSpeedCalculator::velocityCallback, this);
}

void WheelSpeedCalculator::calcAndPublishWheelSpeeds(const geometry_msgs::Twist& velocity_msg)
{
  calcAllWheelSpeeds(velocity_msg);
  publishWheelSpeeds();
}

void WheelSpeedCalculator::calcAllWheelSpeeds(const geometry_msgs::Twist& velocity_msg)
{
  wheel_speeds_msg_.header.stamp = ros::Time::now();
  if (wheel_config_type_ == util::WheelConfigurationType::OMNI_WHEELS ||
      wheel_config_type_ == util::WheelConfigurationType::SKID_STEERING)
  {
    wheel_speeds_msg_.front_left_radians_per_sec = front_left_wheel_.calcWheelSpeed(velocity_msg);
    wheel_speeds_msg_.front_right_radians_per_sec = front_right_wheel_.calcWheelSpeed(velocity_msg);
  }

  wheel_speeds_msg_.back_left_radians_per_sec = back_left_wheel_.calcWheelSpeed(velocity_msg);
  wheel_speeds_msg_.back_right_radians_per_sec = back_right_wheel_.calcWheelSpeed(velocity_msg);
}

void WheelSpeedCalculator::publishWheelSpeeds()
{
  wheel_speeds_pub_.publish(wheel_speeds_msg_);
}

FrontLeftWheelSpeedCalculator::FrontLeftWheelSpeedCalculator(util::WheelConfigurationType wheel_config_type)
  : BaseWheelSpeedCalculator(wheel_config_type)
{
}

float FrontLeftWheelSpeedCalculator::calcWheelSpeed(const geometry_msgs::Twist& cmd_vel_msg)
{
  float wheel_front_left = 0.0;
  switch (wheel_config_type_)
  {
    case util::WheelConfigurationType::OMNI_WHEELS:
      wheel_front_left =
          (1 / util::WHEEL_RADIUS) * (cmd_vel_msg.linear.x - cmd_vel_msg.linear.y -
                                      (util::WHEEL_SEP_WIDTH + util::WHEEL_SEP_LENGTH) * cmd_vel_msg.angular.z);
      break;
    case util::WheelConfigurationType::SKID_STEERING:
      wheel_front_left = (cmd_vel_msg.linear.x - cmd_vel_msg.angular.z * util::WHEEL_SEP_WIDTH / 2.0);
      wheel_front_left = util::convert_meters_per_sec_to_radians_per_sec(wheel_front_left);
      break;
    case util::WheelConfigurationType::DIFFERENTIAL_DRIVE:
      wheel_front_left = 0.0;
      break;
    default:
      break;
  }

  return wheel_front_left;
}

FrontRightWheelSpeedCalculator::FrontRightWheelSpeedCalculator(util::WheelConfigurationType wheel_config_type)
  : BaseWheelSpeedCalculator(wheel_config_type)
{
}

float FrontRightWheelSpeedCalculator::calcWheelSpeed(const geometry_msgs::Twist& cmd_vel_msg)
{
  float wheel_front_right = 0.0;
  switch (wheel_config_type_)
  {
    case util::WheelConfigurationType::OMNI_WHEELS:
      wheel_front_right =
          (1 / util::WHEEL_RADIUS) * (cmd_vel_msg.linear.x + cmd_vel_msg.linear.y +
                                      (util::WHEEL_SEP_WIDTH + util::WHEEL_SEP_LENGTH) * cmd_vel_msg.angular.z);
      break;
    case util::WheelConfigurationType::SKID_STEERING:
      wheel_front_right = (cmd_vel_msg.linear.x + cmd_vel_msg.angular.z * util::WHEEL_SEP_WIDTH / 2.0);
      wheel_front_right = util::convert_meters_per_sec_to_radians_per_sec(wheel_front_right);
      break;
    case util::WheelConfigurationType::DIFFERENTIAL_DRIVE:
      wheel_front_right = 0.0;
      break;
    default:
      break;
  }
  return wheel_front_right;
}

BackRightWheelSpeedCalculator::BackRightWheelSpeedCalculator(util::WheelConfigurationType wheel_config_type)
  : BaseWheelSpeedCalculator(wheel_config_type)
{
}

float BackRightWheelSpeedCalculator::calcWheelSpeed(const geometry_msgs::Twist& cmd_vel_msg)
{
  float wheel_back_right = 0.0;
  switch (wheel_config_type_)
  {
    case util::WheelConfigurationType::OMNI_WHEELS:
      wheel_back_right =
          (1 / util::WHEEL_RADIUS) * (cmd_vel_msg.linear.x - cmd_vel_msg.linear.y +
                                      (util::WHEEL_SEP_WIDTH + util::WHEEL_SEP_LENGTH) * cmd_vel_msg.angular.z);
      break;
    case util::WheelConfigurationType::SKID_STEERING:
      wheel_back_right = (cmd_vel_msg.linear.x + cmd_vel_msg.angular.z * util::WHEEL_SEP_WIDTH / 2.0);
      wheel_back_right = util::convert_meters_per_sec_to_radians_per_sec(wheel_back_right);
      break;
    case util::WheelConfigurationType::DIFFERENTIAL_DRIVE:
      wheel_back_right = (cmd_vel_msg.linear.x + cmd_vel_msg.angular.z * util::WHEEL_SEP_WIDTH / 2.0);
      wheel_back_right = util::convert_meters_per_sec_to_radians_per_sec(wheel_back_right);
      break;
    case util::WheelConfigurationType::FixedPWMSpeeds:
      wheel_back_right = (cmd_vel_msg.linear.x + cmd_vel_msg.angular.z * util::WHEEL_SEP_WIDTH / 2.0);
      wheel_back_right = util::convert_meters_per_sec_to_radians_per_sec(wheel_back_right);
      break;
    default:
      break;
  }
  return wheel_back_right;
}

BackLeftWheelSpeedCalculator::BackLeftWheelSpeedCalculator(util::WheelConfigurationType wheel_config_type)
  : BaseWheelSpeedCalculator(wheel_config_type)
{
}

float BackLeftWheelSpeedCalculator::calcWheelSpeed(const geometry_msgs::Twist& cmd_vel_msg)
{
  float wheel_back_left = 0.0;
  switch (wheel_config_type_)
  {
    case util::WheelConfigurationType::OMNI_WHEELS:
      wheel_back_left =
          (1 / util::WHEEL_RADIUS) * (cmd_vel_msg.linear.x + cmd_vel_msg.linear.y -
                                      (util::WHEEL_SEP_WIDTH + util::WHEEL_SEP_LENGTH) * cmd_vel_msg.angular.z);
      break;
    case util::WheelConfigurationType::SKID_STEERING:
      wheel_back_left = (cmd_vel_msg.linear.x - cmd_vel_msg.angular.z * util::WHEEL_SEP_WIDTH / 2.0);
      wheel_back_left = util::convert_meters_per_sec_to_radians_per_sec(wheel_back_left);
      break;
    case util::WheelConfigurationType::DIFFERENTIAL_DRIVE:
      wheel_back_left = (cmd_vel_msg.linear.x - cmd_vel_msg.angular.z * util::WHEEL_SEP_WIDTH / 2.0);
      wheel_back_left = util::convert_meters_per_sec_to_radians_per_sec(wheel_back_left);
      break;
    case util::WheelConfigurationType::FixedPWMSpeeds:
      wheel_back_left = (cmd_vel_msg.linear.x - cmd_vel_msg.angular.z * util::WHEEL_SEP_WIDTH / 2.0);
      wheel_back_left = util::convert_meters_per_sec_to_radians_per_sec(wheel_back_left);
      break;
    default:
      break;
  }
  return wheel_back_left;
}

}  // namespace wheel_speed_calculator
