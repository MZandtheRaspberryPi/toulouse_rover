#include "toulouse_rover/wheel_speed_controller.h"

namespace wheel_speed_controller
{
static volatile double globalEncCounter[4];
static volatile double globalSpeedCounter[4];

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
  setupPubsSubs(nh, wheel_namespace_);
  priorEncoderCounts_ = 0;
  control_effort_ = 0.;
  encoderStartTime_ = ros::Time::now();

  if (use_pid_)
  {
    std::vector<std::string> nodes;
    bool found_pid_node = false;
    std::string node_to_find = wheel_namespace_ + wheel_namespace_.substr(0, wheel_namespace_.size() - 3);
    while (ros::ok() && !found_pid_node)
    {
      ros::master::getNodes(nodes);
      for (const auto& node : nodes)
      {
        if (node == node_to_find)
        {
          found_pid_node = true;
        }
      }
      ROS_DEBUG("waiting for %s", node_to_find.c_str());
      ros::Duration(0.1).sleep();
    }
  }

  is_initialized_ = true;
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
  state_pub_ = nh.advertise<std_msgs::Float64>(wheel_namespace + "/state", 100);

  set_pub_ = nh.advertise<std_msgs::Float64>(wheel_namespace + "/setpoint", 100);
  if (use_pid_)
  {
    ctrl_sub_ =
        nh.subscribe(wheel_namespace + "/control_effort", 1, &BaseWheelSpeedController::controlEffortCallback, this);
    pid_enable_pub_ = nh.advertise<std_msgs::Bool>(wheel_namespace + "/pid_enable", 100);
    std_msgs::Bool enable_msg;
    enable_msg.data = true;
    pid_enable_pub_.publish(enable_msg);
    //  ros::spinOnce();
  }
}

BaseWheelSpeedController::~BaseWheelSpeedController()
{
  if (use_pid_)
  {
    /*
    std_msgs::Float64 state;
    state.data = 0.0;
    state_pub_.publish(state);
    ros::spinOnce();
*/
    std_msgs::Bool enable_msg;
    enable_msg.data = false;
    pid_enable_pub_.publish(enable_msg);
    // ros::spinOnce();
  }
}

void BaseWheelSpeedController::controlEffortCallback(const std_msgs::Float64& control_effort_input)
{
  ROS_DEBUG("got ctrl effort %s %f", wheel_namespace_.c_str(), control_effort_input.data);
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
    // TODO: Figure out why PID node sometimes publishes zero, even when it shouldnt.
    if (control_effort == 0.)
    {
      return false;
    }
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
      ROS_DEBUG("%s set to 0 setpoint, exiting.", wheel_namespace_.c_str());
      return control_effort;
    }

    bool got_ctrl_effort = false;

    ros::Duration state_pub_increment = ros::Duration(0.25);
    ros::Time last_pub_time = ros::Time::now();
    while (ros::ok() && !got_ctrl_effort)
    {
      // TODO: Figure out why it is not always a 1:1 relationship, state msg to control effort with PID Node.
      if ((ros::Time::now() - last_pub_time) > state_pub_increment)
      {
        publishWheelState();
        last_pub_time = ros::Time::now();
      }
      got_ctrl_effort = getControlEffort(control_effort);
      // ros::spinOnce();
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
  ROS_DEBUG("%s wheel control effort is now: %d", wheel_namespace_.c_str(), control_effort_pwm);
  return control_effort_pwm;
}

void BaseWheelSpeedController::publishSetpoint(const float& speed_radians_per_sec)
{
  ROS_DEBUG("%s wheel setpoint is now: %f", wheel_namespace_.c_str(), speed_radians_per_sec);
  setpoint_ = speed_radians_per_sec;
  if (speed_radians_per_sec == 0.)
  {
    return;
  }

  std_msgs::Float64 setpoint;
  setpoint.data = speed_radians_per_sec;
  set_pub_.publish(setpoint);
  // ros::spinOnce();
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
  ROS_DEBUG("%s wheel state is now: %f", wheel_namespace_.c_str(), current_wheel_speed_);
  state_pub_.publish(state);
  // ros::spinOnce();
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
  bool is_initialized_ = false;

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
  : wheel_namespace_(wheel_namespace), use_pid_(use_pid), wheel_config_type_(wheel_config_type), loop_rate_(loop_rate)
{
  setupCustomSignalHandlers();
  setupGlobalCounters();
  setupGPIO();
  setupServoArray();
  setupPubsSubs(nh, wheel_namespace_);
  enableMotors();
  if (wheel_config_type_ == util::WheelConfigurationType::OMNI_WHEELS ||
      wheel_config_type_ == util::WheelConfigurationType::SKID_STEERING)
  {
    front_right_speed_ctrl_ = new FrontRightWheelSpeedController(
        nh, wheel_namespace_ + "/front_right_wheel" + (use_pid ? "_pid_ns" : ""), use_pid, wheel_config_type);

    front_left_speed_ctrl_ = new FrontLeftWheelSpeedController(
        nh, wheel_namespace_ + "/front_left_wheel" + (use_pid ? "_pid_ns" : ""), use_pid, wheel_config_type);
  }

  back_right_speed_ctrl_ = new BackRightWheelSpeedController(
      nh, wheel_namespace_ + "/back_right_wheel" + (use_pid ? "_pid_ns" : ""), use_pid, wheel_config_type);
  back_left_speed_ctrl_ = new BackLeftWheelSpeedController(
      nh, wheel_namespace_ + "/back_left_wheel" + (use_pid ? "_pid_ns" : ""), use_pid, wheel_config_type);
}

WheelSpeedController::~WheelSpeedController()
{
  if (wheel_config_type_ == util::WheelConfigurationType::OMNI_WHEELS ||
      wheel_config_type_ == util::WheelConfigurationType::SKID_STEERING)
  {
    delete front_right_speed_ctrl_;
    delete front_left_speed_ctrl_;
  }

  delete back_right_speed_ctrl_;
  delete back_left_speed_ctrl_;
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
  servos_absolute_pub_ = nh.advertise<i2cpwm_board::ServoArray>("servos_absolute", 100);

  encoder_pub_ = nh.advertise<toulouse_rover::WheelEncoderCounts>("/" + wheel_namespace_ + "/wheel_adj_enc_count", 100);

  wheel_speed_actual_pub_ =
      nh.advertise<toulouse_rover::WheelSpeeds>("/" + wheel_namespace_ + "/wheel_speeds_actual", 100);
}

void WheelSpeedController::wheelSpeedCallback(const toulouse_rover::WheelSpeeds::ConstPtr& wheel_speeds)
{
  ROS_WARN("getting wheel speeds");
  const std::lock_guard<std::mutex> lock(speedUpdateMutex);
  ROS_WARN("got wheel speeds");
  wheel_speeds_ = *wheel_speeds;

  if (wheel_config_type_ == util::WheelConfigurationType::OMNI_WHEELS ||
      wheel_config_type_ == util::WheelConfigurationType::SKID_STEERING)
  {
    globalSpeedCounter[front_left_speed_ctrl_->encoderIndex_] = wheel_speeds_.front_left_radians_per_sec;
    globalSpeedCounter[front_right_speed_ctrl_->encoderIndex_] = wheel_speeds_.front_right_radians_per_sec;
  }

  globalSpeedCounter[back_right_speed_ctrl_->encoderIndex_] = wheel_speeds_.back_right_radians_per_sec;
  globalSpeedCounter[back_left_speed_ctrl_->encoderIndex_] = wheel_speeds_.back_left_radians_per_sec;
}

void WheelSpeedController::setupGlobalCounters()
{
  const std::lock_guard<std::mutex> speed_lock(speedUpdateMutex);
  const std::lock_guard<std::mutex> fl_lock(frontLeftEncoderMutex);
  const std::lock_guard<std::mutex> fr_lock(frontRightEncoderMutex);
  const std::lock_guard<std::mutex> br_lock(backRightEncoderMutex);
  const std::lock_guard<std::mutex> bl_lock(backLeftEncoderMutex);
  for (int i = 0; i < sizeof(globalSpeedCounter) / sizeof(globalSpeedCounter[0]); i++)
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
    front_left_speed_ctrl_->publishSetpoint(wheel_spds_to_ctrl.front_left_radians_per_sec);
    front_right_speed_ctrl_->publishSetpoint(wheel_spds_to_ctrl.front_right_radians_per_sec);
  }
  back_right_speed_ctrl_->publishSetpoint(wheel_spds_to_ctrl.back_right_radians_per_sec);
  back_left_speed_ctrl_->publishSetpoint(wheel_spds_to_ctrl.back_left_radians_per_sec);
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
    front_left_speed_ctrl_->publishWheelState();
    current_speeds.front_left_radians_per_sec = front_left_speed_ctrl_->getWheelSpeed();

    front_right_speed_ctrl_->publishWheelState();
    current_speeds.front_right_radians_per_sec = front_right_speed_ctrl_->getWheelSpeed();
  }
  back_right_speed_ctrl_->publishWheelState();
  current_speeds.back_left_radians_per_sec = back_right_speed_ctrl_->getWheelSpeed();
  back_left_speed_ctrl_->publishWheelState();
  current_speeds.back_right_radians_per_sec = back_left_speed_ctrl_->getWheelSpeed();

  wheel_speed_actual_pub_.publish(current_speeds);
}

controlEffort WheelSpeedController::get_control_efforts()
{
  controlEffort control_efforts{};
  if (wheel_config_type_ == util::WheelConfigurationType::OMNI_WHEELS ||
      wheel_config_type_ == util::WheelConfigurationType::SKID_STEERING)
  {
    control_efforts.front_left_control_effort = front_left_speed_ctrl_->spinAndWaitForCtrlEffort();
    control_efforts.front_right_control_effort = front_right_speed_ctrl_->spinAndWaitForCtrlEffort();
  }

  control_efforts.back_right_control_effort = back_right_speed_ctrl_->spinAndWaitForCtrlEffort();
  control_efforts.back_left_control_effort = back_left_speed_ctrl_->spinAndWaitForCtrlEffort();
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

  ROS_DEBUG("back right controller: %f", servo_array_.servos[10].value);
  servos_absolute_pub_.publish(servo_array_);
}

void WheelSpeedController::publishAdjEncoderData()
{
  const std::lock_guard<std::mutex> fl_lock(frontLeftEncoderMutex);
  const std::lock_guard<std::mutex> fr_lock(frontRightEncoderMutex);
  const std::lock_guard<std::mutex> br_lock(backRightEncoderMutex);
  const std::lock_guard<std::mutex> bl_lock(backLeftEncoderMutex);
  toulouse_rover::WheelEncoderCounts wheel_adj_encoder_counts;
  if (wheel_config_type_ == util::WheelConfigurationType::OMNI_WHEELS ||
      wheel_config_type_ == util::WheelConfigurationType::SKID_STEERING)
  {
    wheel_adj_encoder_counts.front_left_adjusted_encoder_count =
        globalEncCounter[front_left_speed_ctrl_->encoderIndex_];
    wheel_adj_encoder_counts.front_right_adjusted_encoder_count =
        globalEncCounter[front_right_speed_ctrl_->encoderIndex_];
  }
  else
  {
    wheel_adj_encoder_counts.front_left_adjusted_encoder_count = 0.0;
    wheel_adj_encoder_counts.front_right_adjusted_encoder_count = 0.0;
  }

  wheel_adj_encoder_counts.back_right_adjusted_encoder_count = globalEncCounter[back_right_speed_ctrl_->encoderIndex_];
  wheel_adj_encoder_counts.back_left_adjusted_encoder_count = globalEncCounter[back_left_speed_ctrl_->encoderIndex_];
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

void WheelSpeedController::spinOnce()
{
  {
  const std::lock_guard<std::mutex> lock(speedUpdateMutex);
  publishWheelSetpoints(wheel_speeds_);
  publishAdjEncoderData();
  publishWheelStates();
  }
  controlEffort control_effort = get_control_efforts();
  updateAndPublishServoArray(control_effort);
  loop_rate_.sleep();
}

void WheelSpeedController::spin()
{
  ros::AsyncSpinner spinner(2);
  spinner.start();
  while (!wheel_speed_controller::g_request_shutdown)
  {
    spinOnce();
  }

  zeroOutMotors();
  disableMotors();
}

}  // end namespace wheel_speed_controller
