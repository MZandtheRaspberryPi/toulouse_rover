#include <ros/ros.h>

#include <string>

#include "toulouse_rover/wheel_speed_controller.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wheel_speed_controller_node");
  ros::NodeHandle nh;

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
  ROS_INFO("wheel_speed_controller_node spinning");
  wheel_controller.spin();
}
