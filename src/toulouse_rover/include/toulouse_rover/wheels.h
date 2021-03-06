#ifndef wheels_h
#define wheels_h

#ifdef RPI
#include <wiringPi.h>
#endif
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <cmath>
#include <queue>

#include "odom_calc.h"
#include "util.h"

namespace wheels {
constexpr float MAX_PWM = 4000;
constexpr float MIN_PWM = 900;
constexpr float MAX_PID_CONTROL = 1;
constexpr float MIN_PID_CONTROL = 0;
constexpr float SLOPE_PID =
    (MAX_PWM - MIN_PWM) / (MAX_PID_CONTROL - MIN_PID_CONTROL);
static volatile double globalEncCounter[4];
static volatile double globalSpeedCounter[4];

// MAX I saw, with wheels not on ground, was about 60 encoder ticks per second
// this would translate to 3 rotations, given 20 ticks per rotation, which would
// translate to 2 * PI * RADIUS * 3 ~= .452304 meters in one second
// TODO: Calculate MIN Speed by setting wheels to min PWM and seeing how fast
// they go in meters per sec
float const MAX_ROBOT_SPEED = 2 * M_PI * util::WHEEL_RADIUS * 3;
float const MIN_ROBOT_SPEED = 0;
float const SLOPE_ROBOT_SPEED =
    (MAX_PWM - MIN_PWM) / (MAX_ROBOT_SPEED - MIN_ROBOT_SPEED);

// MAX I saw, with wheels not on ground, was about 60 encoder ticks per second,
// so 3 rotations, so 3 PI. we will get this by going about .22 meters per
// second in x direction, so this is our max speed
// TODO: Calculate MIN Speed by setting wheels to min PWM and seeing how fast
// they go in radians
constexpr float const& MAX_WHEEL_SPEED = 3 * M_PI;
constexpr float const& MIN_WHEEL_SPEED = 0.;
constexpr float const& SLOPE_WHEEL_SPEED =
    (MAX_PWM - MIN_PWM) / (MAX_WHEEL_SPEED - MIN_WHEEL_SPEED);

void genericInterupt(int index);
void frontLeftInterupt();
void frontRightInterupt();
void backRightInterupt();
void backLeftInterupt();

class FrontLeftWheel;

class FrontRightWheel;

class BackRightWheel;

class BackLeftWheel;

class WheelController {
 public:
  WheelController(ros::NodeHandle& nh, std::string wheel_namespace,
                  bool use_pid, odom_calc::OdomType odom_type);
  void setupPubsSubs(ros::NodeHandle& nh, std::string wheel_namespace);

  void controlEffortCallback(const std_msgs::Float64& control_effort_input);

  // scaling the output from PID, into PWM space and returning PWM signal value
  // for wheel
  int getPWM();

  int setRadPerSec(int radPerSec);

  int update();

  int getPWMCtrlEff(float control_effort);

  void pubSpeedError();
  void pubEncCounts();
  int ctrlWheelPID(float speed);
  static void wheelInterupt();

  double getEncoderCounts();
  double getWheelSpeed();

  virtual float calcWheelSpeed(const geometry_msgs::Twist& cmd_vel_msg) = 0;
  int pwmFromWheelSpeed(float wheel_speed);
  int ctrlWheelCmdVel(const geometry_msgs::Twist& cmd_vel_msg);

  friend FrontLeftWheel;
  friend FrontRightWheel;
  friend BackRightWheel;
  friend BackLeftWheel;

 private:
  int encoderIndex_;
  int encTicksPerRotation_{20};
  bool oppositeSide_;

  float control_effort_;
  double priorEncoderCounts_ = 0;
  int interuptPin_;
  std::string wheel_namespace_;

  ros::Time encoderStartTime_;

  ros::Publisher state_pub_;
  ros::Publisher set_pub_;
  ros::Subscriber ctrl_sub_;
  odom_calc::OdomType odom_type_;
  double CHECK_RATE_CTRL{60};
  bool use_pid_;
};

class FrontLeftWheel : public WheelController {
 public:
  FrontLeftWheel(ros::NodeHandle& nh, std::string wheel_namespace, bool use_pid,
                 odom_calc::OdomType odom_type);
  float calcWheelSpeed(const geometry_msgs::Twist& cmd_vel_msg);
};

class FrontRightWheel : public WheelController {
 public:
  FrontRightWheel(ros::NodeHandle& nh, std::string wheel_namespace,
                  bool use_pid, odom_calc::OdomType odom_type);
  float calcWheelSpeed(const geometry_msgs::Twist& cmd_vel_msg);
};

class BackRightWheel : public WheelController {
 public:
  BackRightWheel(ros::NodeHandle& nh, std::string wheel_namespace, bool use_pid,
                 odom_calc::OdomType odom_type);
  float calcWheelSpeed(const geometry_msgs::Twist& cmd_vel_msg);
};

class BackLeftWheel : public WheelController {
 public:
  BackLeftWheel(ros::NodeHandle& nh, std::string wheel_namespace, bool use_pid,
                odom_calc::OdomType odom_type);
  float calcWheelSpeed(const geometry_msgs::Twist& cmd_vel_msg);
};

}  // end namespace wheels

#endif