#include "wheels.h"

namespace wheels {
void genericInterupt(int index) {
  if (globalSpeedCounter[index] > 0) {
    globalEncCounter[index]++;
  } else if (globalSpeedCounter[index] == 0.0) {
    return;
  } else {
    globalEncCounter[index]--;
  }
}

void frontLeftInterupt() { genericInterupt(0); }

void frontRightInterupt() { genericInterupt(1); }
void backRightInterupt() { genericInterupt(2); }

void backLeftInterupt() { genericInterupt(3); }

WheelController::WheelController(ros::NodeHandle& nh,
                                 std::string wheel_namespace, bool use_pid,
                                 odom_calc::OdomType odom_type)
    : wheel_namespace_(wheel_namespace),
      use_pid_(use_pid),
      odom_type_(odom_type) {
  setupPubsSubs(nh, wheel_namespace);
  priorEncoderCounts_ = 0;
  control_effort_ = 0.;
}

void WheelController::setupPubsSubs(ros::NodeHandle& nh,
                                    std::string wheel_namespace) {
  state_pub_ =
      nh.advertise<std_msgs::Float64>("/" + wheel_namespace + "/state", 1);

  set_pub_ =
      nh.advertise<std_msgs::Float64>("/" + wheel_namespace + "/setpoint", 1);
  if (use_pid_) {
    ctrl_sub_ = nh.subscribe("/" + wheel_namespace + "/control_effort", 1,
                             &WheelController::controlEffortCallback, this);
  }
}

void WheelController::controlEffortCallback(
    const std_msgs::Float64& control_effort_input) {
  control_effort_ = control_effort_input.data;
}

void WheelController::pubSpeedError() {
  std_msgs::Float64 state;
  double currentEncoderCounts = globalEncCounter[encoderIndex_];
  ROS_INFO("%f", currentEncoderCounts);
  double changeEncoderCounts = currentEncoderCounts - priorEncoderCounts_;
  priorEncoderCounts_ = currentEncoderCounts;
  ros::Time now = ros::Time::now();
  ros::Duration elapsed_time_s = now - encoderStartTime_;
  encoderStartTime_ = now;
  ROS_INFO("%s: change in encoder count: %f", wheel_namespace_.c_str(),
           changeEncoderCounts);
  // state.data = changeEncoderCounts * 2 * M_PI / encTicksPerRotation_ /
  // elapsed_time_s.toSec(); // 2 pi radians is 20 encoder counts, so this will
  // give us radians per second
  state.data = changeEncoderCounts / elapsed_time_s.toSec();
  state_pub_.publish(state);
  ros::spinOnce();
}

void WheelController::pubEncCounts() {
  std_msgs::Float64 counts;
  counts.data = globalEncCounter[encoderIndex_];
  state_pub_.publish(counts);
  ros::spinOnce();
}

int WheelController::ctrlWheelPID(float speed) {
  if (speed != globalSpeedCounter[encoderIndex_]) {
    globalSpeedCounter[encoderIndex_] = speed;
  }

  if (speed == 0.) {
    return 0;
  }

  std_msgs::Float64 setpoint;
  setpoint.data = speed;
  set_pub_.publish(setpoint);
  ros::spinOnce();
  ros::Duration(0.01)
      .sleep();  // giving PID time to get this setpoint before publishing state

  pubSpeedError();

  int pwm = getPWMCtrlEff(control_effort_);

  return pwm;
}

int WheelController::getPWMCtrlEff(float control_effort) {
  ROS_INFO("ctrl: %f SLP: %f ", control_effort, SLOPE_PID);
  float scaled_control_effort =
      MIN_PWM + SLOPE_PID * (control_effort - MIN_PID_CONTROL);
  if (globalSpeedCounter[encoderIndex_] < 0) {
    scaled_control_effort *= -1;
  }
  int pwm = static_cast<int>(scaled_control_effort);
  pwm = std::min(static_cast<int>(MAX_PID_CONTROL), pwm);
  return pwm;
}

double WheelController::getEncoderCounts() {
  return globalEncCounter[encoderIndex_];
}

int WheelController::pwmFromWheelSpeed(float wheel_speed) {
  if (wheel_speed == 0.) {
    return 0;
  }

  float scaled_wheel_pwm =
      MIN_PWM + SLOPE_WHEEL_SPEED * (std::abs(wheel_speed) - MIN_WHEEL_SPEED);

  if (scaled_wheel_pwm > MAX_PWM) {
    ROS_WARN("Wheel commanded to %f pwm, capping to the max of %f",
             scaled_wheel_pwm, MAX_PWM);
    scaled_wheel_pwm = MAX_PWM;
  }

  if (wheel_speed < 0) {
    scaled_wheel_pwm *= -1;
  }
  int pwm = static_cast<int>(scaled_wheel_pwm);
  return pwm;
}

int WheelController::ctrlWheelCmdVel(const geometry_msgs::Twist& cmd_vel_msg) {
  float wheel_speed = calcWheelSpeed(cmd_vel_msg);
  globalSpeedCounter[encoderIndex_] = wheel_speed;
  std_msgs::Float64 setpoint;
  setpoint.data = wheel_speed;
  set_pub_.publish(setpoint);
  pubEncCounts();
  int pwm = pwmFromWheelSpeed(wheel_speed);
  return pwm;
}

double WheelController::getWheelSpeed() {
  double currentEncoderCounts = globalEncCounter[encoderIndex_];
  double changeEncoderCounts = currentEncoderCounts - priorEncoderCounts_;
  priorEncoderCounts_ = currentEncoderCounts;
  ros::Time now = ros::Time::now();
  ros::Duration elapsed_time_s = now - encoderStartTime_;
  encoderStartTime_ = now;
  ROS_INFO("%s: change in encoder count: %f", wheel_namespace_.c_str(),
           changeEncoderCounts);
  double speedRadiansPerSec =
      changeEncoderCounts * 2 * M_PI / encTicksPerRotation_ /
      elapsed_time_s.toSec();  // 2 pi radians is 20 encoder counts, so this
                               // will give us radians per second
  return speedRadiansPerSec;
}

FrontLeftWheel::FrontLeftWheel(ros::NodeHandle& nh, std::string wheel_namespace,
                               bool use_pid, odom_calc::OdomType odom_type)
    : WheelController(nh, wheel_namespace, use_pid, odom_type) {
  encoderIndex_ = 0;
}

float FrontLeftWheel::calcWheelSpeed(const geometry_msgs::Twist& cmd_vel_msg) {
  float wheel_front_left = 0.0;
  switch (odom_type_) {
    case odom_calc::OMNI_WHEELS:
      wheel_front_left =
          (1 / util::WHEEL_RADIUS) *
          (cmd_vel_msg.linear.x - cmd_vel_msg.linear.y -
           (util::WHEEL_SEP_WIDTH + util::WHEEL_SEP_LENGTH) * cmd_vel_msg.angular.z);
      break;
    case odom_calc::SKID_STEERING:
      wheel_front_left = (cmd_vel_msg.linear.x -
                          cmd_vel_msg.angular.z * util::WHEEL_SEP_WIDTH / 2.0) /
                         util::WHEEL_RADIUS;
      wheel_front_left =
          util::convert_meters_per_sec_to_radians_per_sec(wheel_front_left);
      break;
    default:
      break;
  }

  return wheel_front_left;
}

FrontRightWheel::FrontRightWheel(ros::NodeHandle& nh,
                                 std::string wheel_namespace, bool use_pid,
                                 odom_calc::OdomType odom_type)
    : WheelController(nh, wheel_namespace, use_pid, odom_type) {
  encoderIndex_ = 1;
}

float FrontRightWheel::calcWheelSpeed(const geometry_msgs::Twist& cmd_vel_msg) {
  float wheel_front_right = 0.0;
  switch (odom_type_) {
    case odom_calc::OMNI_WHEELS:
      wheel_front_right =
          (1 / util::WHEEL_RADIUS) *
          (cmd_vel_msg.linear.x + cmd_vel_msg.linear.y +
           (util::WHEEL_SEP_WIDTH + util::WHEEL_SEP_LENGTH) * cmd_vel_msg.angular.z);
      break;
    case odom_calc::SKID_STEERING:
      wheel_front_right = (cmd_vel_msg.linear.x +
                           cmd_vel_msg.angular.z * util::WHEEL_SEP_WIDTH / 2.0) /
                          util::WHEEL_RADIUS;
      wheel_front_right =
          util::convert_meters_per_sec_to_radians_per_sec(wheel_front_right);
      break;
    default:
      break;
  }
  return wheel_front_right;
}

BackRightWheel::BackRightWheel(ros::NodeHandle& nh, std::string wheel_namespace,
                               bool use_pid, odom_calc::OdomType odom_type)
    : WheelController(nh, wheel_namespace, use_pid, odom_type) {
  encoderIndex_ = 2;
}

float BackRightWheel::calcWheelSpeed(const geometry_msgs::Twist& cmd_vel_msg) {
  float wheel_back_right = 0.0;
  switch (odom_type_) {
    case odom_calc::OMNI_WHEELS:
      wheel_back_right =
          (1 / util::WHEEL_RADIUS) *
          (cmd_vel_msg.linear.x - cmd_vel_msg.linear.y +
           (util::WHEEL_SEP_WIDTH + util::WHEEL_SEP_LENGTH) * cmd_vel_msg.angular.z);
      break;
    case odom_calc::SKID_STEERING:
      wheel_back_right = (cmd_vel_msg.linear.x +
                          cmd_vel_msg.angular.z * util::WHEEL_SEP_WIDTH / 2.0) /
                         util::WHEEL_RADIUS;
      wheel_back_right =
          util::convert_meters_per_sec_to_radians_per_sec(wheel_back_right);
      break;
    default:
      break;
  }
  return wheel_back_right;
}

BackLeftWheel::BackLeftWheel(ros::NodeHandle& nh, std::string wheel_namespace,
                             bool use_pid, odom_calc::OdomType odom_type)
    : WheelController(nh, wheel_namespace, use_pid, odom_type) {
  encoderIndex_ = 3;
}

float BackLeftWheel::calcWheelSpeed(const geometry_msgs::Twist& cmd_vel_msg) {
  float wheel_back_left = 0.0;
  switch (odom_type_) {
    case odom_calc::OMNI_WHEELS:
      wheel_back_left =
          (1 / util::WHEEL_RADIUS) *
          (cmd_vel_msg.linear.x + cmd_vel_msg.linear.y -
           (util::WHEEL_SEP_WIDTH + util::WHEEL_SEP_LENGTH) * cmd_vel_msg.angular.z);
      break;
    case odom_calc::SKID_STEERING:
      wheel_back_left = (cmd_vel_msg.linear.x -
                         cmd_vel_msg.angular.z * util::WHEEL_SEP_WIDTH / 2.0) /
                        util::WHEEL_RADIUS;
      wheel_back_left =
          util::convert_meters_per_sec_to_radians_per_sec(wheel_back_left);
      break;
    default:
      break;
  }
  return wheel_back_left;
}

}  // end namespace wheels
