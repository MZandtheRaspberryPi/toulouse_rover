#include "toulouse_rover/wheel_speed_controller.h"

namespace wheel_speed_controller
{
static volatile double globalEncCounter[4];
static volatile double globalSpeedCounter[4];

toulouse_rover::WheelPwmSpeeds getZeroPwmSpeedsMsg()
{
  toulouse_rover::WheelPwmSpeeds pwm_speeds_msg;
  pwm_speeds_msg.motor_enable = true;
  pwm_speeds_msg.front_left_pwm_1 = 0;
  pwm_speeds_msg.front_left_pwm_2 = 0;

  pwm_speeds_msg.front_right_pwm_1 = 0;
  pwm_speeds_msg.front_right_pwm_2 = 0;

  pwm_speeds_msg.back_left_pwm_1 = 0;
  pwm_speeds_msg.back_left_pwm_2 = 0;

  pwm_speeds_msg.back_right_pwm_1 = 0;
  pwm_speeds_msg.back_right_pwm_2 = 0;

  return pwm_speeds_msg;
}

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
  const std::lock_guard<std::mutex> enc_lock(encoderUpdateMutex);
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
  state.data = changeEncoderCounts * 2 * M_PI / (elapsed_time_s.toSec() * util::ENCODER_TICKS_PER_ROTATION);
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

  bool is_wheel_backwards_rotation = wheel_speed < 0;
  wheel_speed = std::abs(wheel_speed);

  if (wheel_speed < util::MIN_WHEEL_SPEED)
  {
    ROS_WARN("%s commanded to %f (absolute) radians per sec, flooring to the min of %f. Backwards flag is: %d.",
             wheel_namespace_.c_str(), wheel_speed, util::MIN_WHEEL_SPEED, is_wheel_backwards_rotation);
    wheel_speed = util::MIN_WHEEL_SPEED;
  }
  else if (wheel_speed > util::MAX_WHEEL_SPEED)
  {
    ROS_WARN("%s commanded to %f (absolute) radians per sec, capping to the max of %f. Backwards flag is: %d.",
             wheel_namespace_.c_str(), wheel_speed, util::MAX_WHEEL_SPEED, is_wheel_backwards_rotation);
    wheel_speed = util::MAX_WHEEL_SPEED;
  }

  float scaled_wheel_pwm = util::MIN_PWM + util::SLOPE_WHEEL_SPEED * (std::abs(wheel_speed) - util::MIN_WHEEL_SPEED);

  if (is_wheel_backwards_rotation)
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

  toulouse_rover::WheelEncoderCounts initial_enc_counts;
  initial_enc_counts.back_left_encoder_count = 0;
  initial_enc_counts.back_right_encoder_count = 0;
  initial_enc_counts.front_left_encoder_count = 0;
  initial_enc_counts.front_right_encoder_count = 0;
  prior_enc_counts_ = initial_enc_counts;
  setFixedPwmSpeeds(controlEffort{}, controlEffort{}, controlEffort{}, controlEffort{});
}

void WheelSpeedController::setFixedPwmSpeeds(controlEffort forward, controlEffort backward, controlEffort left,
                                             controlEffort right)
{
  FixedPWMDirectionsAndSpeeds fixed_pwm_speeds{};
  fixed_pwm_speeds.forward = forward;
  fixed_pwm_speeds.backward = backward;
  fixed_pwm_speeds.left = left;
  fixed_pwm_speeds.right = right;
  fixed_pwm_speeds_ = fixed_pwm_speeds;
}

controlEffort WheelSpeedController::getCtrlEffortForFixedPwm(toulouse_rover::WheelSpeeds wheel_speeds)
{
  controlEffort ctrl_effort{};
  // forward
  if (wheel_speeds.back_left_radians_per_sec > 0 && wheel_speeds.back_right_radians_per_sec > 0)
  {
    ctrl_effort.back_left_control_effort = fixed_pwm_speeds_.forward.back_left_control_effort;
    ctrl_effort.back_right_control_effort = fixed_pwm_speeds_.forward.back_right_control_effort;
  }
  // left
  else if (wheel_speeds.back_left_radians_per_sec < 0 && wheel_speeds.back_right_radians_per_sec > 0)
  {
    ctrl_effort.back_left_control_effort = fixed_pwm_speeds_.left.back_left_control_effort;
    ctrl_effort.back_right_control_effort = fixed_pwm_speeds_.left.back_right_control_effort;
  }
  // right
  else if (wheel_speeds.back_left_radians_per_sec > 0 && wheel_speeds.back_right_radians_per_sec < 0)
  {
    ctrl_effort.back_left_control_effort = fixed_pwm_speeds_.right.back_left_control_effort;
    ctrl_effort.back_right_control_effort = fixed_pwm_speeds_.right.back_right_control_effort;
  }
  // back
  else
  {
    ctrl_effort.back_left_control_effort = fixed_pwm_speeds_.backward.back_left_control_effort;
    ctrl_effort.back_right_control_effort = fixed_pwm_speeds_.backward.back_right_control_effort;
  }
  return ctrl_effort;
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

void WheelSpeedController::setupPubsSubs(ros::NodeHandle& nh, const std::string wheel_namespace)
{
  wheel_speed_sub_ =
      nh.subscribe("/" + wheel_namespace_ + "/wheel_cmd_speeds", 1, &WheelSpeedController::wheelSpeedCallback, this);
  raw_encoder_sub_ =
      nh.subscribe("/" + wheel_namespace_ + "/wheel_raw_enc_count", 1, &WheelSpeedController::rawEncoderCallback, this);

  encoder_pub_ = nh.advertise<toulouse_rover::WheelEncoderCounts>("/" + wheel_namespace_ + "/wheel_adj_enc_count", 100);

  wheel_speed_actual_pub_ =
      nh.advertise<toulouse_rover::WheelSpeeds>("/" + wheel_namespace_ + "/wheel_speeds_actual", 100);
  wheel_pwm_pub_ = nh.advertise<toulouse_rover::WheelPwmSpeeds>("/" + wheel_namespace_ + "/wheels_pwm_cmd", 100);
}

void WheelSpeedController::wheelSpeedCallback(const toulouse_rover::WheelSpeeds::ConstPtr& wheel_speeds)
{
  ROS_DEBUG("getting wheel speeds");
  const std::lock_guard<std::mutex> lock(speedUpdateMutex);
  ROS_DEBUG("got wheel speeds");
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
  for (int i = 0; i < sizeof(globalSpeedCounter) / sizeof(globalSpeedCounter[0]); i++)
  {
    globalSpeedCounter[i] = 0.0;
    globalEncCounter[i] = 0.0;
  }
}

void WheelSpeedController::enableMotors()
{
  toulouse_rover::WheelPwmSpeeds pwm_msg = getZeroPwmSpeedsMsg();
  pwm_msg.motor_enable = true;
  wheel_pwm_pub_.publish(pwm_msg);
}

void WheelSpeedController::disableMotors()
{
  toulouse_rover::WheelPwmSpeeds pwm_msg = getZeroPwmSpeedsMsg();
  pwm_msg.motor_enable = false;
  wheel_pwm_pub_.publish(pwm_msg);
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
  current_speeds.back_right_radians_per_sec = back_right_speed_ctrl_->getWheelSpeed();
  back_left_speed_ctrl_->publishWheelState();
  current_speeds.back_left_radians_per_sec = back_left_speed_ctrl_->getWheelSpeed();

  wheel_speed_actual_pub_.publish(current_speeds);
}

controlEffort WheelSpeedController::get_control_efforts()
{
  controlEffort control_efforts{};

  if (wheel_config_type_ == util::WheelConfigurationType::FixedPWMSpeeds)
  {
    return getCtrlEffortForFixedPwm(wheel_speeds_);
  }

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

void WheelSpeedController::publishWheelPwm(const controlEffort& control_effort)
{
  toulouse_rover::WheelPwmSpeeds pwm_speeds_msg;
  pwm_speeds_msg.motor_enable = true;
  if (control_effort.front_left_control_effort < 0)
  {
    pwm_speeds_msg.front_left_pwm_1 = 0;
    pwm_speeds_msg.front_left_pwm_2 = std::abs(control_effort.front_left_control_effort);
  }
  else
  {
    pwm_speeds_msg.front_left_pwm_1 = std::abs(control_effort.front_left_control_effort);
    pwm_speeds_msg.front_left_pwm_2 = 0;
  }

  if (control_effort.front_right_control_effort < 0)
  {
    pwm_speeds_msg.front_right_pwm_1 = 0;
    pwm_speeds_msg.front_right_pwm_2 = std::abs(control_effort.front_right_control_effort);
  }
  else
  {
    pwm_speeds_msg.front_right_pwm_1 = std::abs(control_effort.front_right_control_effort);
    pwm_speeds_msg.front_right_pwm_2 = 0;
  }

  if (control_effort.back_left_control_effort < 0)
  {
    pwm_speeds_msg.back_left_pwm_1 = 0;
    pwm_speeds_msg.back_left_pwm_2 = std::abs(control_effort.back_left_control_effort);
  }
  else
  {
    pwm_speeds_msg.back_left_pwm_1 = std::abs(control_effort.back_left_control_effort);
    pwm_speeds_msg.back_left_pwm_2 = 0;
  }

  if (control_effort.back_right_control_effort < 0)
  {
    pwm_speeds_msg.back_right_pwm_1 = 0;
    pwm_speeds_msg.back_right_pwm_2 = std::abs(control_effort.back_right_control_effort);
  }
  else
  {
    pwm_speeds_msg.back_right_pwm_1 = std::abs(control_effort.back_right_control_effort);
    pwm_speeds_msg.back_right_pwm_2 = 0;
  }
  wheel_pwm_pub_.publish(pwm_speeds_msg);
}

// we need to filter out bad points that we sometimes get
// we also need to add negative sign depending on the wheel speed
double adjustEncoderData(const double& raw_count, const double& prior_raw_count,
                         const float& max_encoder_ticks_per_cycle, const int& pwm_speed_1, const int& pwm_speed_2,
                         const float& wheel_speed)
{
  double adjusted_count = raw_count;
  if (raw_count > max_encoder_ticks_per_cycle && prior_raw_count < max_encoder_ticks_per_cycle)
  {
    adjusted_count = prior_raw_count;
  }
  else if (raw_count > max_encoder_ticks_per_cycle && prior_raw_count >= max_encoder_ticks_per_cycle)
  {
    adjusted_count = 0;
  }

  if (pwm_speed_1 == 0 && pwm_speed_2 == 0)
  {
    adjusted_count = 0;
  }

  if (wheel_speed < 0)
  {
    adjusted_count = adjusted_count * (-1);
  }

  return adjusted_count;
}

void WheelSpeedController::rawEncoderCallback(const toulouse_rover::WheelEncoderCounts::ConstPtr& encoder_msg)
{
  const std::lock_guard<std::mutex> enc_lock(encoderUpdateMutex);
  const std::lock_guard<std::mutex> speed_lock(speedUpdateMutex);

  double prior_raw_count;
  double raw_count;
  double adjusted_counts;
  // sometimes if wheels get stopped at just the right spot the ir encoders flip out and generate a lot of ticks
  // hence bounding it to a reasonable number
  float max_encoder_ticks_per_cycle =
      loop_rate_.expectedCycleTime().toSec() * util::MAX_WHEEL_SPEED * util::ENCODER_TICKS_PER_ROTATION / (2 * M_PI);

  if (wheel_config_type_ == util::WheelConfigurationType::OMNI_WHEELS ||
      wheel_config_type_ == util::WheelConfigurationType::SKID_STEERING)
  {
    prior_raw_count = prior_enc_counts_.front_left_encoder_count;
    raw_count = encoder_msg->front_left_encoder_count;
    adjusted_counts =
        adjustEncoderData(raw_count, prior_raw_count, max_encoder_ticks_per_cycle, encoder_msg->back_left_pwm_1,
                          encoder_msg->back_left_pwm_2, wheel_speeds_.front_left_radians_per_sec);
    globalEncCounter[front_left_speed_ctrl_->encoderIndex_] =
        globalEncCounter[front_left_speed_ctrl_->encoderIndex_] + adjusted_counts;

    prior_raw_count = prior_enc_counts_.front_right_encoder_count;
    raw_count = encoder_msg->front_right_encoder_count;
    adjusted_counts =
        adjustEncoderData(raw_count, prior_raw_count, max_encoder_ticks_per_cycle, encoder_msg->back_left_pwm_1,
                          encoder_msg->back_left_pwm_2, wheel_speeds_.front_right_radians_per_sec);
    globalEncCounter[front_right_speed_ctrl_->encoderIndex_] += adjusted_counts;
  }

  prior_raw_count = prior_enc_counts_.back_left_encoder_count;
  raw_count = encoder_msg->back_left_encoder_count;
  adjusted_counts =
      adjustEncoderData(raw_count, prior_raw_count, max_encoder_ticks_per_cycle, encoder_msg->back_left_pwm_1,
                        encoder_msg->back_left_pwm_2, wheel_speeds_.back_left_radians_per_sec);
  globalEncCounter[back_left_speed_ctrl_->encoderIndex_] += adjusted_counts;

  prior_raw_count = prior_enc_counts_.back_right_encoder_count;
  raw_count = encoder_msg->back_right_encoder_count;
  adjusted_counts =
      adjustEncoderData(raw_count, prior_raw_count, max_encoder_ticks_per_cycle, encoder_msg->back_right_pwm_1,
                        encoder_msg->back_right_pwm_2, wheel_speeds_.back_right_radians_per_sec);
  globalEncCounter[back_right_speed_ctrl_->encoderIndex_] += adjusted_counts;

  prior_enc_counts_ = *encoder_msg;
}

void WheelSpeedController::publishAdjEncoderData()
{
  const std::lock_guard<std::mutex> enc_lock(encoderUpdateMutex);
  toulouse_rover::WheelEncoderCounts wheel_adj_encoder_counts;
  if (wheel_config_type_ == util::WheelConfigurationType::OMNI_WHEELS ||
      wheel_config_type_ == util::WheelConfigurationType::SKID_STEERING)
  {
    wheel_adj_encoder_counts.front_left_encoder_count = globalEncCounter[front_left_speed_ctrl_->encoderIndex_];
    wheel_adj_encoder_counts.front_right_encoder_count = globalEncCounter[front_right_speed_ctrl_->encoderIndex_];
  }
  else
  {
    wheel_adj_encoder_counts.front_left_encoder_count = 0.0;
    wheel_adj_encoder_counts.front_right_encoder_count = 0.0;
  }

  wheel_adj_encoder_counts.back_right_encoder_count = globalEncCounter[back_right_speed_ctrl_->encoderIndex_];
  wheel_adj_encoder_counts.back_left_encoder_count = globalEncCounter[back_left_speed_ctrl_->encoderIndex_];
  encoder_pub_.publish(wheel_adj_encoder_counts);
}

void WheelSpeedController::zeroOutMotors()
{
  controlEffort ctrl_effort;
  ctrl_effort.front_left_control_effort = 0;
  ctrl_effort.front_right_control_effort = 0;
  ctrl_effort.back_left_control_effort = 0;
  ctrl_effort.back_right_control_effort = 0;
  publishWheelPwm(ctrl_effort);
}

void WheelSpeedController::spinOnce()
{
  ros::spinOnce();
  controlEffort control_effort;
  {
    const std::lock_guard<std::mutex> speed_lock(speedUpdateMutex);
    publishWheelSetpoints(wheel_speeds_);
    publishAdjEncoderData();
    publishWheelStates();

    control_effort = get_control_efforts();
  }
  publishWheelPwm(control_effort);

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
