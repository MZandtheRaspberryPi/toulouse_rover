#include "toulouse_rover/util.h"

namespace util {

float convert_radians_per_sec_to_meters_per_sec(float radians_per_sec) {
  // radians per second * revolutions per radian * meters per revolution
  return (radians_per_sec / (2 * M_PI)) * WHEEL_CIRCUMFERENCE;
}

float convert_meters_per_sec_to_radians_per_sec(float meters_per_sec) {
  // meters per second * revolution per meter * radians per revolation
  return (meters_per_sec / WHEEL_CIRCUMFERENCE) * 2 * M_PI;
}

bool validate_wheel_config_type(const std::string& wheel_config_type) {
  bool valid = false;
  for (int i = 0; i < POSSIBLE_WHEEL_CONFIGURATION_STRS.size(); i++) {
    if (wheel_config_type == POSSIBLE_WHEEL_CONFIGURATION_STRS[i]) {
      valid = true;
    }
  }
  return valid;
}

std::string get_valid_wheel_config_types() {
  std::string wheel_config_types{};
  for (int i = 0; i < POSSIBLE_WHEEL_CONFIGURATION_STRS.size(); i++) {
    wheel_config_types += POSSIBLE_WHEEL_CONFIGURATION_STRS[i] + "\n";
  }
  return wheel_config_types;
}

WheelConfigurationType getWheelConfigEnumFromStr(
    const std::string& wheel_config_type) {
  for (int i = 0; i < POSSIBLE_WHEEL_CONFIGURATION_STRS.size(); i++) {
    if (wheel_config_type == POSSIBLE_WHEEL_CONFIGURATION_STRS[i]) {
      return static_cast<WheelConfigurationType>(i);
    }
  }
  return static_cast<WheelConfigurationType>(-1);
}

bool getWheelConfigTypeFromNH(ros::NodeHandle& nh,
                              WheelConfigurationType& wheel_config_type) {
  std::string wheel_config_type_str;
  if (!nh.getParam("/wheel_config_type", wheel_config_type_str)) {
    ROS_FATAL(
        "param /wheel_config_type is not set and must be to calculate wheel "
        "speeds.");
    return false;
  }

  if (!validate_wheel_config_type(wheel_config_type_str)) {
    ROS_FATAL(
        "/wheel_config_type is not a valid option. %s passed and options are: "
        "%s",
        wheel_config_type, get_valid_wheel_config_types());
    return false;
  }

  wheel_config_type = getWheelConfigEnumFromStr(wheel_config_type_str);
  return true;
}

}  // end namespace util