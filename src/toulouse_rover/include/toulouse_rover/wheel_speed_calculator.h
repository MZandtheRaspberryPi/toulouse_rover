#ifndef wheel_calculator_h
#define wheel_calculator_h

#include <geometry_msgs/Twist.h>
#include <math.h>
#include <ros/ros.h>

#include <cmath>
#include <mutex>
#include <string>

#include "toulouse_rover/WheelSpeeds.h"
#include "toulouse_rover/util.h"

namespace wheel_speed_calculator
{
class BaseWheelSpeedCalculator
{
public:
  BaseWheelSpeedCalculator(util::WheelConfigurationType wheel_config_type);
  virtual float calcWheelSpeed(const geometry_msgs::Twist& cmd_vel_msg) = 0;
  util::WheelConfigurationType wheel_config_type_;
};

class FrontLeftWheelSpeedCalculator : public BaseWheelSpeedCalculator
{
public:
  FrontLeftWheelSpeedCalculator(util::WheelConfigurationType wheel_config_type);
  float calcWheelSpeed(const geometry_msgs::Twist& cmd_vel_msg);
};

class FrontRightWheelSpeedCalculator : public BaseWheelSpeedCalculator
{
public:
  FrontRightWheelSpeedCalculator(util::WheelConfigurationType wheel_config_type);
  float calcWheelSpeed(const geometry_msgs::Twist& cmd_vel_msg);
};

class BackRightWheelSpeedCalculator : public BaseWheelSpeedCalculator
{
public:
  BackRightWheelSpeedCalculator(util::WheelConfigurationType wheel_config_type);
  float calcWheelSpeed(const geometry_msgs::Twist& cmd_vel_msg);
};

class BackLeftWheelSpeedCalculator : public BaseWheelSpeedCalculator
{
public:
  BackLeftWheelSpeedCalculator(util::WheelConfigurationType wheel_config_type);
  float calcWheelSpeed(const geometry_msgs::Twist& cmd_vel_msg);
};

class WheelSpeedCalculator
{
public:
  WheelSpeedCalculator(ros::NodeHandle& nh, const std::string wheel_namespace,
                       util::WheelConfigurationType wheel_config_type);
  void setupPubsSubs(ros::NodeHandle& nh, const std::string wheel_namespace);

  void calcAndPublishWheelSpeeds(const geometry_msgs::Twist& velocity_msg);
  void publishWheelSpeeds();
  void calcAllWheelSpeeds(const geometry_msgs::Twist& velocity_msg);

private:
  void velocityCallback(const geometry_msgs::Twist::ConstPtr& vel);
  std::mutex vel_update_mtx_;
  std::string wheel_namespace_;
  geometry_msgs::Twist latest_vel_msg_;
  ros::Publisher wheel_speeds_pub_;
  ros::Subscriber ctrl_sub_;
  util::WheelConfigurationType wheel_config_type_;

  toulouse_rover::WheelSpeeds wheel_speeds_msg_;

  FrontLeftWheelSpeedCalculator front_left_wheel_;
  FrontRightWheelSpeedCalculator front_right_wheel_;
  BackRightWheelSpeedCalculator back_right_wheel_;
  BackLeftWheelSpeedCalculator back_left_wheel_;
};

}  // namespace wheel_speed_calculator

#endif