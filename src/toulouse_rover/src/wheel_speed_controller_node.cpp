#include <ros/ros.h>

#include <string>

#include "toulouse_rover/wheel_speed_controller.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "wheel_speed_controller_node");
  ros::NodeHandle nh;

  std::string wheel_namespace_str;
  util::WheelConfigurationType wheel_config_type;

  bool got_param_bool = util::getWheelConfigTypeFromNH(nh, wheel_config_type);
  if (!got_param_bool) {
    ROS_FATAL("couldn't get wheel config type, exiting....");
    exit(1);
  }

  if (!nh.getParam("/wheel_namespace", wheel_namespace_str)) {
    ROS_WARN("param /wheel_namespace is not set.");
  }

  wheel_speed_controller::WheelSpeedController wheel_controller(
      nh, wheel_namespace_str, wheel_config_type);
  ros::spin();
}