#include <ros/ros.h>

#include <string>

#include "toulouse_rover/odom_calculator.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_calc_node");
  ros::NodeHandle nh;

  std::string wheel_namespace_str;
  util::WheelConfigurationType wheel_config_type;

  bool got_param_bool = util::getWheelConfigTypeFromNH(nh, wheel_config_type);
  if (!got_param_bool)
  {
    ROS_FATAL(
        "wheel_speed_calculator_node: couldn't get wheel config type, "
        "exiting....");
    exit(1);
  }

  odom_calculator::OdomCalculator odom_calc(nh, wheel_config_type);
  ros::spin();
}