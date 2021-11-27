#ifdef RPI
#include <wiringPi.h>
#endif
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <cmath> 
#include <queue>

constexpr float MAX_PWM = 4000;
constexpr float MIN_PWM = 900;
constexpr float MAX_PID_CONTROL = 1;
constexpr float MIN_PID_CONTROL = 0;
constexpr float SLOPE_PID = (MAX_PWM - MIN_PWM) / (MAX_PID_CONTROL - MIN_PID_CONTROL);
static volatile double globalEncCounter [4];
static volatile double globalSpeedCounter [4];

static constexpr int const& WHEEL_SEP_LENGTH = .130; // how far wheels are apart length meters
static constexpr int const& WHEEL_SEP_WIDTH = .092; // how far wheels are apart width meters
static constexpr int const& WHEEL_RADIUS = .024; // radius of wheels meters

// MAX I saw, with wheels not on ground, was about 60 encoder ticks per second
// this would translate to 3 rotations, given 20 ticks per rotation, which would translate to
// 2 * PI * RADIUS * 3 ~= .452304 meters in one second
static constexpr float const& MAX_ROBOT_SPEED = 2 * M_PI * WHEEL_RADIUS * 3;
static constexpr float const& MIN_ROBOT_SPEED = 0.;
constexpr float SLOPE_ROBOT_SPEED = (MAX_PWM - MIN_PWM) / (MAX_ROBOT_SPEED - MIN_ROBOT_SPEED);

// MAX I saw, with wheels not on ground, was about 60 encoder ticks per second, so 3 rotations, so 3 PI.
static constexpr float const& MAX_WHEEL_SPEED = 3 * M_PI;
static constexpr float const& MIN_WHEEL_SPEED = 0.;
constexpr float SLOPE_WHEEL_SPEED = (MAX_PWM - MIN_PWM) / (MAX_WHEEL_SPEED - MIN_WHEEL_SPEED);

void genericInterupt(int index);
void frontLeftInterupt();
void frontRightInterupt();
void backRightInterupt();
void backLeftInterupt();

class WheelController {

public:

  WheelController(ros::NodeHandle& nh, std::string wheel_namespace, bool use_pid);
  void setupPubsSubs(ros::NodeHandle& nh, std::string wheel_namespace);

  void controlEffortCallback(const std_msgs::Float64& control_effort_input);

  // scaling the output from PID, into PWM space and returning PWM signal value for wheel
  int getPWM();

  int setRadPerSec(int radPerSec);

  int update();

  int getPWMCtrlEff(float control_effort);

  void pubSpeedError();
  int ctrlWheelPID(float speed);
  int ctrlWheelCmdVel(geometry_msgs::Twist cmd_vel_msg);
  static void wheelInterupt();

  double getEncoderCounts();

  virtual float calcWheelSpeed(const geometry_msgs::Twist cmd_vel_msg);
  int pwmFromWheelSpeed(float wheel_speed);
  int ctrlWheelCmdVel(const geometry_msgs::Twist cmd_vel_msg);

private:

  int encoderIndex_;
  int encTicksPerRotation_ {20};
  bool oppositeSide_;

  float control_effort_;
  double priorEncoderCounts_ = 0;
  int interuptPin_;
  std::string wheel_namespace_;

  ros::Time encoderStartTime_;

  ros::Publisher state_pub_;
  ros::Publisher set_pub_;
  ros::Subscriber ctrl_sub_;
  double CHECK_RATE_CTRL{60};
  bool use_pid_;
};

class FrontLeftWheel : public WheelController
{
public:
  FrontLeftWheel(ros::NodeHandle& nh, std::string wheel_namespace, bool use_pid);
  int getPWMSpeed(float speed);
};

class FrontRightWheel : public WheelController
{
public:
  FrontLeftWheel(ros::NodeHandle& nh, std::string wheel_namespace, bool use_pid);
  int getPWMSpeed(float speed);
};

class BackRightWheel : public WheelController
{
public:
  FrontLeftWheel(ros::NodeHandle& nh, std::string wheel_namespace, bool use_pid);
  int getPWMSpeed(float speed); 
};

class BackLeftWheel : public WheelController
{
public:
  FrontLeftWheel(ros::NodeHandle& nh, std::string wheel_namespace, bool use_pid);
  int getPWMSpeed(float speed);
};