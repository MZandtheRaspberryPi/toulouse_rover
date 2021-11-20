#ifdef RPI
#include <wiringPi.h>
#endif
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <queue>

constexpr float MAX_PWM = 4000;
constexpr float MIN_PWM = 900;
constexpr float MAX_PID_CONTROL = 100;
constexpr float MIN_PID_CONTROL = 0;
constexpr float SLOPE = (MAX_PWM - MIN_PWM) / (MAX_PID_CONTROL - MIN_PID_CONTROL);

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
  

  int const& WHEEL_RADIUS = 24; // radius of wheels
private:

  void wheelInterupt();
  int encTicksPerRotation_ {20};
  bool oppositeSide_;

  std::queue<float> control_effort_;
  volatile int priorEncoderCounts_ = 0;
  volatile int encoderCounts_ = 0;
  int interuptPin_;
  float commandRadPerSec_;
  std::string wheel_namespace_;

  ros::Time encoderStartTime_;

  ros::Publisher state_pub_;
  ros::Publisher set_pub_;
  ros::Subscriber ctrl_sub_;
  ros::Rate loop_rate_;
  double CHECK_RATE_CTRL{60};
};
