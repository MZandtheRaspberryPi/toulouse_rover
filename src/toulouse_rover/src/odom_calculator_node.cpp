#include <ros/ros.h>

#include <string>

#include "toulouse_rover/odom_calculator.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_calc_node");
  ros::NodeHandle nh;

  std::string wheel_namespace_str;
  std::string odom_frame_id;
  std::string odom_topic;
  util::WheelConfigurationType wheel_config_type;

  bool got_param_bool = util::getWheelConfigTypeFromNH(nh, wheel_config_type);
  if (!got_param_bool)
  {
    ROS_FATAL(
        "wheel_speed_calculator_node: couldn't get wheel config type, "
        "exiting....");
    exit(1);
  }

  if (!nh.getParam("/wheel_namespace", wheel_namespace_str))
  {
    ROS_WARN("odom_calculator_node: param /wheel_namespace is not set.");
  }

  if (!nh.getParam("odom_frame_id", odom_frame_id))
  {
    ROS_WARN("odom_calculator_node: param odom_frame_id is not set. defaulting it");
    odom_frame_id = "odom_wheels";
  }

  if (!nh.getParam("odom_topic", odom_topic))
  {
    ROS_WARN("odom_calculator_node: param odom_topic is not set. defaulting it");
    odom_topic = "odom_wheels";
  }


  odom_calculator::OdomCalculator odom_calc(nh, wheel_config_type, wheel_namespace_str, odom_topic, odom_frame_id);
  ros::spin();
}
