#ifndef wheel_controller_h
#define wheel_controller_h

#include <signal.h>
#ifdef RPI
#include <wiringPi.h>
#endif
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <mutex>

#include "i2cpwm_board/Servo.h"
#include "i2cpwm_board/ServoArray.h"

// TODO: MZ, put wheel objects into controlle class?

#include <ros/xmlrpc_manager.h>

#include <cmath>

#include "toulouse_rover/WheelEncoderCounts.h"
#include "toulouse_rover/WheelSpeeds.h"
#include "util.h"

namespace wheel_speed_controller
{
// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig);

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);

static volatile double globalEncCounter[4];
static volatile double globalSpeedCounter[4];

std::mutex speedUpdateMutex;
std::mutex frontLeftEncoderMutex;
std::mutex frontRightEncoderMutex;
std::mutex backLeftEncoderMutex;
std::mutex backRightEncoderMutex;

void genericInterupt(int index);
void frontLeftInterupt();
void frontRightInterupt();
void backRightInterupt();
void backLeftInterupt();

struct controlEffort
{
  float front_left_control_effort;
  float front_right_control_effort;
  float back_right_control_effort;
  float back_left_control_effort;
};

class BaseWheelSpeedController
{
public:
  BaseWheelSpeedController(ros::NodeHandle& nh, std::string wheel_namespace, bool use_pid,
                           util::WheelConfigurationType wheel_config_type);
  int encoderIndex_;
  float getWheelSpeed();
  void publishSetpoint(const float& speed_radians_per_sec);
  void publishWheelState();
  int spinAndWaitForCtrlEffort();
  int pwmFromWheelSpeed(float wheel_speed);
  bool getControlEffort(float& control_effort);

private:
  void setupPubsSubs(ros::NodeHandle& nh, const std::string wheel_namespace);
  void controlEffortCallback(const std_msgs::Float64& control_effort_input);

  util::WheelConfigurationType wheel_config_type_;
  ros::Publisher state_pub_;
  ros::Publisher set_pub_;
  ros::Subscriber ctrl_sub_;

  std::string wheel_namespace_;
  bool use_pid_;
  bool got_control_data_;
  ros::Time encoderStartTime_;
  float control_effort_;
  float setpoint_;
  float current_wheel_speed_;
  std::mutex control_effort_mutex_;
  double priorEncoderCounts_ = 0;
  const float time_to_wait_for_control_msg_{ 0.01 };
};

class FrontLeftWheelSpeedController : public BaseWheelSpeedController
{
public:
  FrontLeftWheelSpeedController(ros::NodeHandle& nh, std::string wheel_namespace, bool use_pid,
                                util::WheelConfigurationType wheel_config_type);
  void publishWheelState();
};

class FrontRightWheelSpeedController : public BaseWheelSpeedController
{
public:
  FrontRightWheelSpeedController(ros::NodeHandle& nh, std::string wheel_namespace, bool use_pid,
                                 util::WheelConfigurationType wheel_config_type);
  void publishWheelState();
};

class BackRightWheelSpeedController : public BaseWheelSpeedController
{
public:
  BackRightWheelSpeedController(ros::NodeHandle& nh, std::string wheel_namespace, bool use_pid,
                                util::WheelConfigurationType wheel_config_type);
  void publishWheelState();
};

class BackLeftWheelSpeedController : public BaseWheelSpeedController
{
public:
  BackLeftWheelSpeedController(ros::NodeHandle& nh, std::string wheel_namespace, bool use_pid,
                               util::WheelConfigurationType wheel_config_type);
  void publishWheelState();
};

class WheelSpeedController
{
public:
  WheelSpeedController(ros::NodeHandle& nh, std::string wheel_namespace, bool use_pid,
                       util::WheelConfigurationType wheel_config_type, float loop_rate);

  void enableMotors();
  void disableMotors();
  void spin();

private:
  void setupCustomSignalHandlers();
  void setupPubsSubs(ros::NodeHandle& nh, const std::string wheel_namespace);
  void setupGlobalCounters();
  void setupServoArray();
  void setupGPIO();
  void setupWheels(const util::WheelConfigurationType& wheel_config_type);
  void wheelSpeedCallback(const toulouse_rover::WheelSpeeds::ConstPtr& wheel_speeds);
  void publishWheelSetpoints(const toulouse_rover::WheelSpeeds& wheel_spds_to_ctrl);
  void zeroOutMotors();
  void publishWheelStates();
  void publishAdjEncoderData();

  controlEffort get_control_efforts();
  void updateAndPublishServoArray(const controlEffort& control_effort);
  ros::Rate loop_rate_;

  std::string wheel_namespace_;

  i2cpwm_board::ServoArray servo_array_{};

  ros::Subscriber wheel_speed_sub_;
  ros::Publisher servos_absolute_pub_;
  ros::Publisher encoder_pub_;
  ros::Publisher wheel_speed_actual_pub_;
  toulouse_rover::WheelSpeeds wheel_speeds_;
  util::WheelConfigurationType wheel_config_type_;
  bool use_pid_;

  FrontLeftWheelSpeedController front_left_speed_ctrl_;
  FrontRightWheelSpeedController front_right_speed_ctrl_;
  BackRightWheelSpeedController back_right_speed_ctrl_;
  BackLeftWheelSpeedController back_left_speed_ctrl_;
};

}  // namespace wheel_speed_controller

#endif