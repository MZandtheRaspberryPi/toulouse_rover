#include <ros/ros.h>

#include <string>

#include "toulouse_rover/wheel_speed_controller.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wheel_speed_controller_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string wheel_namespace_str;
  util::WheelConfigurationType wheel_config_type;

  bool got_param_bool = util::getWheelConfigTypeFromNH(nh, wheel_config_type);
  if (!got_param_bool)
  {
    ROS_FATAL("couldn't get wheel config type, exiting....");
    exit(1);
  }

  if (!nh.getParam("/wheel_namespace", wheel_namespace_str))
  {
    ROS_WARN("wheel_speed_controller_node: param /wheel_namespace is not set.");
  }

  float loop_rate = 10.0;
  if (!nh.getParam("speed_controller_loop_rate", loop_rate))
  {
    ROS_WARN(
        "wheel_speed_controller_node: param speed_controller_loop_rate is not "
        "set.");
  }

  bool use_pid = false;
  if (!nh.getParam("speed_controller_use_pid", use_pid))
  {
    ROS_WARN(
        "wheel_speed_controller_node: param speed_controller_use_pid is not "
        "set.");
  }

  wheel_speed_controller::WheelSpeedController wheel_controller(nh, wheel_namespace_str, use_pid, wheel_config_type,
                                                                loop_rate);

  if (wheel_config_type == util::WheelConfigurationType::FixedPWMSpeeds)
  {
    wheel_speed_controller::controlEffort forward{};
    nh_private.getParam("fixed_pwm_speeds/forward/back_left", forward.back_left_control_effort);
    nh_private.getParam("fixed_pwm_speeds/forward/back_right", forward.back_right_control_effort);
    wheel_speed_controller::controlEffort backward{};
    nh_private.getParam("fixed_pwm_speeds/backward/back_left", backward.back_left_control_effort);
    nh_private.getParam("fixed_pwm_speeds/backward/back_right", backward.back_right_control_effort);
    wheel_speed_controller::controlEffort left{};
    nh_private.getParam("fixed_pwm_speeds/left/back_left", left.back_left_control_effort);
    nh_private.getParam("fixed_pwm_speeds/left/back_right", left.back_right_control_effort);
    wheel_speed_controller::controlEffort right{};
    nh_private.getParam("fixed_pwm_speeds/right/back_left", right.back_left_control_effort);
    nh_private.getParam("fixed_pwm_speeds/right/back_right", right.back_right_control_effort);

    wheel_controller.setFixedPwmSpeeds(forward, backward, left, right);
  }

  ROS_INFO("wheel_speed_controller_node spinning");
  wheel_controller.spin();
}
