#include "toulouse_rover/wheel_speed_controller.h"

namespace wheel_speed_controller
{
// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  wheel_speed_controller::g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    wheel_speed_controller::g_request_shutdown = 1;  // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

void genericInterupt(int index)
{
  const std::lock_guard<std::mutex> lock(speedUpdateMutex);
  if (wheel_speed_controller::globalSpeedCounter[index] > 0)
  {
    wheel_speed_controller::globalEncCounter[index]++;
  }
  else if (wheel_speed_controller::globalSpeedCounter[index] == 0.0)
  {
    return;
  }
  else
  {
    wheel_speed_controller::globalEncCounter[index]--;
  }
}

void frontLeftInterupt()
{
  const std::lock_guard<std::mutex> lock(frontLeftEncoderMutex);
  genericInterupt(0);
}

void frontRightInterupt()
{
  const std::lock_guard<std::mutex> lock(frontRightEncoderMutex);
  genericInterupt(1);
}
void backRightInterupt()
{
  const std::lock_guard<std::mutex> lock(backRightEncoderMutex);
  genericInterupt(2);
}

void backLeftInterupt()
{
  const std::lock_guard<std::mutex> lock(backLeftEncoderMutex);
  genericInterupt(3);
}

BaseWheelSpeedController::BaseWheelSpeedController(ros::NodeHandle& nh, const std::string wheel_namespace, bool use_pid,
                                                   util::WheelConfigurationType wheel_config_type)
  : wheel_namespace_(wheel_namespace), use_pid_(use_pid), wheel_config_type_(wheel_config_type)
{
  setupPubsSubs(nh, wheel_namespace);
  priorEncoderCounts_ = 0;
  control_effort_ = 0.;
}

FrontLeftWheelSpeedController::FrontLeftWheelSpeedController(ros::NodeHandle& nh, std::string wheel_namespace,
                                                             bool use_pid,
                                                             util::WheelConfigurationType wheel_config_type)
  : BaseWheelSpeedController(nh, wheel_namespace, use_pid, wheel_config_type)
{
  encoderIndex_ = 0;
}
void FrontLeftWheelSpeedController::publishWheelState()
{
  const std::lock_guard<std::mutex> lock(frontLeftEncoderMutex);
  BaseWheelSpeedController::publishWheelState();
}

FrontRightWheelSpeedController::FrontRightWheelSpeedController(ros::NodeHandle& nh, std::string wheel_namespace,
                                                               bool use_pid,
                                                               util::WheelConfigurationType wheel_config_type)
  : BaseWheelSpeedController(nh, wheel_namespace, use_pid, wheel_config_type)
{
  encoderIndex_ = 1;
}
void FrontRightWheelSpeedController::publishWheelState()
{
  const std::lock_guard<std::mutex> lock(frontRightEncoderMutex);
  BaseWheelSpeedController::publishWheelState();
}

BackRightWheelSpeedController::BackRightWheelSpeedController(ros::NodeHandle& nh, std::string wheel_namespace,
                                                             bool use_pid,
                                                             util::WheelConfigurationType wheel_config_type)
  : BaseWheelSpeedController(nh, wheel_namespace, use_pid, wheel_config_type)
{
  encoderIndex_ = 2;
}
void BackRightWheelSpeedController::publishWheelState()
{
  const std::lock_guard<std::mutex> lock(backRightEncoderMutex);
  BaseWheelSpeedController::publishWheelState();
}

BackLeftWheelSpeedController::BackLeftWheelSpeedController(ros::NodeHandle& nh, std::string wheel_namespace,
                                                           bool use_pid, util::WheelConfigurationType wheel_config_type)
  : BaseWheelSpeedController(nh, wheel_namespace, use_pid, wheel_config_type)
{
  encoderIndex_ = 3;
}
void BackLeftWheelSpeedController::publishWheelState()
{
  const std::lock_guard<std::mutex> lock(backLeftEncoderMutex);
  BaseWheelSpeedController::publishWheelState();
}

void BaseWheelSpeedController::setupPubsSubs(ros::NodeHandle& nh, const std::string wheel_namespace)
{
  state_pub_ = nh.advertise<std_msgs::Float64>("/" + wheel_namespace + "/state", 1);

  set_pub_ = nh.advertise<std_msgs::Float64>("/" + wheel_namespace + "/setpoint", 1);

  if (use_pid_)
  {
    ctrl_sub_ = nh.subscribe("/" + wheel_namespace + "/control_effort", 1,
                             &BaseWheelSpeedController::controlEffortCallback, this);
  }
}

void BaseWheelSpeedController::controlEffortCallback(const std_msgs::Float64& control_effort_input)
{
  const std::lock_guard<std::mutex> lock(control_effort_mutex_);
  control_effort_ = control_effort_input.data;
  got_control_data_ = true;
}

bool BaseWheelSpeedController::getControlEffort(float& control_effort)
{
  const std::lock_guard<std::mutex> lock(control_effort_mutex_);
  if (got_control_data_)
  {
    control_effort = control_effort_;
    got_control_data_ = false;
    return true;
  }
  else
  {
    return false;
  }
}

int BaseWheelSpeedController::spinAndWaitForCtrlEffort()
{
  float control_effort = 0.0;
  int control_effort_pwm = 0.0;
  if (use_pid_)
  {
    if (setpoint_ == 0.)
    {
      return control_effort;
    }

    bool got_ctrl_effort = false;

    while (!got_ctrl_effort)
    {
      got_ctrl_effort = getControlEffort(control_effort);
      ros::spinOnce();
      ros::Duration(time_to_wait_for_control_msg_).sleep();
    }

    // todo, do we need to make this negative if speed negative? or does pid
    // allow negatives.
    control_effort_pwm = static_cast<int>(control_effort);
  }
  else
  {
    control_effort_pwm = pwmFromWheelSpeed(setpoint_);
  }
  return control_effort_pwm;
}

void BaseWheelSpeedController::publishSetpoint(const float& speed_radians_per_sec)
{
  setpoint_ = speed_radians_per_sec;
  if (speed_radians_per_sec == 0.)
  {
    return;
  }

  std_msgs::Float64 setpoint;
  setpoint.data = speed_radians_per_sec;
  set_pub_.publish(setpoint);
}

void BaseWheelSpeedController::publishWheelState()
{
  std_msgs::Float64 state;
  double currentEncoderCounts = globalEncCounter[encoderIndex_];
  double changeEncoderCounts = currentEncoderCounts - priorEncoderCounts_;
  priorEncoderCounts_ = currentEncoderCounts;
  ros::Time now = ros::Time::now();
  ros::Duration elapsed_time_s = now - encoderStartTime_;
  encoderStartTime_ = now;

  // state.data = changeEncoderCounts * 2 * M_PI / encTicksPerRotation_ /
  // elapsed_time_s.toSec(); // 2 pi radians is 20 encoder counts, so this will
  // give us radians per second
  state.data = changeEncoderCounts / elapsed_time_s.toSec();
  current_wheel_speed_ = state.data;
  state_pub_.publish(state);
}

float BaseWheelSpeedController::getWheelSpeed()
{
  return current_wheel_speed_;
}

int BaseWheelSpeedController::pwmFromWheelSpeed(float wheel_speed)
{
  if (wheel_speed == 0.)
  {
    return 0;
  }

  float scaled_wheel_pwm = util::MIN_PWM + util::SLOPE_WHEEL_SPEED * (std::abs(wheel_speed) - util::MIN_WHEEL_SPEED);

  if (scaled_wheel_pwm > util::MAX_PWM)
  {
    ROS_WARN("Wheel commanded to %f pwm, capping to the max of %f", scaled_wheel_pwm, util::MAX_PWM);
    scaled_wheel_pwm = util::MAX_PWM;
  }

  if (wheel_speed < 0)
  {
    scaled_wheel_pwm *= -1;
  }
  int pwm = static_cast<int>(scaled_wheel_pwm);
  return pwm;
}

WheelSpeedController::WheelSpeedController(ros::NodeHandle& nh, std::string wheel_namespace, bool use_pid,
                                           util::WheelConfigurationType wheel_config_type, float loop_rate)
  : wheel_namespace_(wheel_namespace)
  , use_pid_(use_pid)
  , wheel_config_type_(wheel_config_type)
  , loop_rate_(loop_rate)
  , front_left_speed_ctrl_(nh, wheel_namespace_ + "/front_left_wheel", use_pid, wheel_config_type)
  , front_right_speed_ctrl_(nh, wheel_namespace_ + "/front_right_wheel", use_pid, wheel_config_type)
  , back_right_speed_ctrl_(nh, wheel_namespace_ + "/back_right_wheel", use_pid, wheel_config_type)
  , back_left_speed_ctrl_(nh, wheel_namespace_ + "/back_left_wheel", use_pid, wheel_config_type)
{
  setupCustomSignalHandlers();
  setupGlobalCounters();
  setupGPIO();
  setupServoArray();
  setupPubsSubs(nh, wheel_namespace_);
  enableMotors();
}

void WheelSpeedController::setupCustomSignalHandlers()
{
  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, mySigIntHandler);

  // Override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);
}

void WheelSpeedController::setupGPIO()
{
#ifdef RPI
  wiringPiSetup();
  pinMode(util::MOTOR_ENABLE_PIN, OUTPUT);
  wiringPiISR(util::FRONT_LEFT_INTERUPT_PIN, INT_EDGE_FALLING, &wheels::frontLeftInterupt);
  wiringPiISR(util::FRONT_RIGHT_INTERUPT_PIN, INT_EDGE_FALLING, &wheels::frontRightInterupt);
  wiringPiISR(util::BACK_LEFT_INTERUPT_PIN, INT_EDGE_FALLING, &wheels::backLeftInterupt);
  wiringPiISR(util::BACK_RIGHT_INTERUPT_PIN, INT_EDGE_FALLING, &wheels::backRightInterupt);
#endif
}

void WheelSpeedController::setupPubsSubs(ros::NodeHandle& nh, const std::string wheel_namespace)
{
  wheel_speed_sub_ =
      nh.subscribe("/" + wheel_namespace_ + "/wheel_cmd_speeds", 1, &WheelSpeedController::wheelSpeedCallback, this);
  // servos_absolute publisher
  ros::Publisher servos_absolute_pub_ = nh.advertise<i2cpwm_board::ServoArray>("servos_absolute", 1);

  encoder_pub_ = nh.advertise<toulouse_rover::WheelEncoderCounts>("wheel_adj_enc_count", 1);

  wheel_speed_actual_pub_ = nh.advertise<toulouse_rover::WheelSpeeds>("wheel_speeds", 1);
}

void WheelSpeedController::wheelSpeedCallback(const toulouse_rover::WheelSpeeds::ConstPtr& wheel_speeds)
{
  const std::lock_guard<std::mutex> lock(speedUpdateMutex);
  wheel_speeds_ = *wheel_speeds;

  globalSpeedCounter[front_left_speed_ctrl_.encoderIndex_] = wheel_speeds_.front_left_radians_per_sec;
  globalSpeedCounter[front_right_speed_ctrl_.encoderIndex_] = wheel_speeds_.front_right_radians_per_sec;
  globalSpeedCounter[back_right_speed_ctrl_.encoderIndex_] = wheel_speeds_.back_right_radians_per_sec;
  globalSpeedCounter[back_left_speed_ctrl_.encoderIndex_] = wheel_speeds_.back_left_radians_per_sec;
}

void WheelSpeedController::setupGlobalCounters()
{
  const std::lock_guard<std::mutex> speed_lock(speedUpdateMutex);
  const std::lock_guard<std::mutex> fl_lock(frontLeftEncoderMutex);
  const std::lock_guard<std::mutex> fr_lock(frontRightEncoderMutex);
  const std::lock_guard<std::mutex> br_lock(backRightEncoderMutex);
  const std::lock_guard<std::mutex> bl_lock(backLeftEncoderMutex);
  for (int i = 0; i < sizeof(globalSpeedCounter) / globalSpeedCounter[0]; i++)
  {
    globalSpeedCounter[i] = 0.0;
    globalEncCounter[i] = 0.0;
  }
}

void WheelSpeedController::setupServoArray()
{
  // Initialize servo array message with 16 servo objects
  for (int i = 1; i <= 16; i++)
  {
    i2cpwm_board::Servo temp_servo;
    temp_servo.servo = i;
    temp_servo.value = 0;
    servo_array_.servos.push_back(temp_servo);
  }
}

void WheelSpeedController::enableMotors()
{
#ifdef RPI
  digitalWrite(util::MOTOR_ENABLE_PIN, HIGH);
#endif
}

void WheelSpeedController::disableMotors()
{
#ifdef RPI
  digitalWrite(util::MOTOR_ENABLE_PIN, LOW);
#endif
}

void WheelSpeedController::publishWheelSetpoints(const toulouse_rover::WheelSpeeds& wheel_spds_to_ctrl)
{
  if (wheel_config_type_ == util::WheelConfigurationType::OMNI_WHEELS ||
      wheel_config_type_ == util::WheelConfigurationType::SKID_STEERING)
  {
    front_left_speed_ctrl_.publishSetpoint(wheel_spds_to_ctrl.front_left_radians_per_sec);
    front_right_speed_ctrl_.publishSetpoint(wheel_spds_to_ctrl.front_right_radians_per_sec);
  }
  back_right_speed_ctrl_.publishSetpoint(wheel_spds_to_ctrl.back_right_radians_per_sec);
  back_left_speed_ctrl_.publishSetpoint(wheel_spds_to_ctrl.back_left_radians_per_sec);
}

void WheelSpeedController::publishWheelStates()
{
  toulouse_rover::WheelSpeeds current_speeds;
  current_speeds.header.stamp = ros::Time::now();

  current_speeds.front_left_radians_per_sec = 0.;
  current_speeds.front_right_radians_per_sec = 0.;
  current_speeds.back_left_radians_per_sec = 0.;
  current_speeds.back_right_radians_per_sec = 0.;

  if (wheel_config_type_ == util::WheelConfigurationType::OMNI_WHEELS ||
      wheel_config_type_ == util::WheelConfigurationType::SKID_STEERING)
  {
    front_left_speed_ctrl_.publishWheelState();
    current_speeds.front_left_radians_per_sec = front_left_speed_ctrl_.getWheelSpeed();

    front_right_speed_ctrl_.publishWheelState();
    current_speeds.front_right_radians_per_sec = front_right_speed_ctrl_.getWheelSpeed();
  }
  back_right_speed_ctrl_.publishWheelState();
  current_speeds.back_left_radians_per_sec = back_right_speed_ctrl_.getWheelSpeed();
  back_left_speed_ctrl_.publishWheelState();
  current_speeds.back_right_radians_per_sec = back_left_speed_ctrl_.getWheelSpeed();

  wheel_speed_actual_pub_.publish(current_speeds);
}

controlEffort WheelSpeedController::get_control_efforts()
{
  controlEffort control_efforts{};
  float front_left_control_effort, front_right_control_effort, back_left_control_effort,
      back_right_control_effort = 0.0;
  if (wheel_config_type_ == util::WheelConfigurationType::OMNI_WHEELS ||
      wheel_config_type_ == util::WheelConfigurationType::SKID_STEERING)
  {
    control_efforts.front_left_control_effort = front_left_speed_ctrl_.spinAndWaitForCtrlEffort();
    control_efforts.front_right_control_effort = front_right_speed_ctrl_.spinAndWaitForCtrlEffort();
  }

  control_efforts.back_right_control_effort = back_right_speed_ctrl_.spinAndWaitForCtrlEffort();
  control_efforts.back_left_control_effort = back_left_speed_ctrl_.spinAndWaitForCtrlEffort();
  return control_efforts;
}

void WheelSpeedController::updateAndPublishServoArray(const controlEffort& control_effort)
{
  if (control_effort.front_left_control_effort < 0)
  {
    servo_array_.servos[14].value = 0;
    servo_array_.servos[15].value = std::abs(control_effort.front_left_control_effort);
  }
  else
  {
    servo_array_.servos[15].value = 0;
    servo_array_.servos[14].value = std::abs(control_effort.front_left_control_effort);
  }

  if (control_effort.front_right_control_effort < 0)
  {
    servo_array_.servos[13].value = 0;
    servo_array_.servos[12].value = std::abs(control_effort.front_right_control_effort);
  }
  else
  {
    servo_array_.servos[12].value = 0;
    servo_array_.servos[13].value = std::abs(control_effort.front_right_control_effort);
  }

  if (control_effort.back_left_control_effort < 0)
  {
    servo_array_.servos[9].value = 0;
    servo_array_.servos[8].value = std::abs(control_effort.back_left_control_effort);
  }
  else
  {
    servo_array_.servos[8].value = 0;
    servo_array_.servos[9].value = std::abs(control_effort.back_left_control_effort);
  }

  if (control_effort.back_right_control_effort < 0)
  {
    servo_array_.servos[10].value = 0;
    servo_array_.servos[11].value = std::abs(control_effort.back_right_control_effort);
  }
  else
  {
    servo_array_.servos[11].value = 0;
    servo_array_.servos[10].value = std::abs(control_effort.back_right_control_effort);
  }

  servos_absolute_pub_.publish(servo_array_);
}

void WheelSpeedController::publishAdjEncoderData()
{
  const std::lock_guard<std::mutex> fl_lock(frontLeftEncoderMutex);
  const std::lock_guard<std::mutex> fr_lock(frontRightEncoderMutex);
  const std::lock_guard<std::mutex> br_lock(backRightEncoderMutex);
  const std::lock_guard<std::mutex> bl_lock(backLeftEncoderMutex);
  toulouse_rover::WheelEncoderCounts wheel_adj_encoder_counts;
  wheel_adj_encoder_counts.front_left_adjusted_encoder_count = globalEncCounter[front_left_speed_ctrl_.encoderIndex_];
  wheel_adj_encoder_counts.front_right_adjusted_encoder_count = globalEncCounter[front_right_speed_ctrl_.encoderIndex_];
  wheel_adj_encoder_counts.back_right_adjusted_encoder_count = globalEncCounter[back_right_speed_ctrl_.encoderIndex_];
  wheel_adj_encoder_counts.back_left_adjusted_encoder_count = globalEncCounter[back_left_speed_ctrl_.encoderIndex_];
  encoder_pub_.publish(wheel_adj_encoder_counts);
}

void WheelSpeedController::zeroOutMotors()
{
  for (int i = 0; i < 16; i++)
  {
    servo_array_.servos[i].value = 0;
  }
  servos_absolute_pub_.publish(servo_array_);
}

void WheelSpeedController::spin()
{
  while (!wheel_speed_controller::g_request_shutdown)
  {
    const std::lock_guard<std::mutex> lock(speedUpdateMutex);
    publishWheelSetpoints(wheel_speeds_);
    publishAdjEncoderData();
    publishWheelStates();

    controlEffort control_effort = get_control_efforts();
    updateAndPublishServoArray(control_effort);

    ros::spinOnce();
    loop_rate_.sleep();
  }

  zeroOutMotors();
  disableMotors();
}

}  // end namespace wheel_speed_controller
