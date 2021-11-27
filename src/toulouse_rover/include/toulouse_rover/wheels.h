#ifdef RPI
#include <wiringPi.h>
#endif
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <queue>

constexpr float MAX_PWM = 4000;
constexpr float MIN_PWM = 900;
constexpr float MAX_PID_CONTROL = 1;
constexpr float MIN_PID_CONTROL = 0;
constexpr float SLOPE = (MAX_PWM - MIN_PWM) / (MAX_PID_CONTROL - MIN_PID_CONTROL);
static volatile double globalEncCounter [4];
static volatile double globalSpeedCounter [4];

void genericInterupt(int index);
void frontLeftInterupt();
void frontRightInterupt();
void backRightInterupt();
void backLeftInterupt();

class WheelController {

public:

  WheelController(ros::NodeHandle& nh, std::string wheel_namespace, int interuptPin);
  void setupPubsSubs(ros::NodeHandle& nh, std::string wheel_namespace);

  void controlEffortCallback(const std_msgs::Float64& control_effort_input);

  // scaling the output from PID, into PWM space and returning PWM signal value for wheel
  int getPWM();

  int setRadPerSec(int radPerSec);

  int update();

  int getPWM(float control_effort);

  void pubSpeedError();
  int ctrlWheel(float speed);
  static void wheelInterupt();  

  int const& WHEEL_RADIUS = 24; // radius of wheels
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
};
