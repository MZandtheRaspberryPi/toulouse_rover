#ifndef util_h
#define util_h


#include <math.h>
#include <vector>
#include <string>
#include <ros/ros.h>

namespace util {

enum WheelConfigurationType { OMNI_WHEELS, SKID_STEERING, DIFFERENTIAL_DRIVE };
const std::vector<std::string> POSSIBLE_WHEEL_CONFIGURATION_STRS{"omni_wheels", "skid_steering", "differential_drive"};

bool validate_wheel_config_type(const std::string & wheel_config_type);
std::string get_valid_wheel_config_types();

WheelConfigurationType getWheelConfigEnumFromStr(const std::string & wheel_config_type);

bool getWheelConfigTypeFromNH(ros::NodeHandle & nh, WheelConfigurationType & wheel_config_type);

#define MOTOR_ENABLE_PIN 22

#define FRONT_LEFT_INTERUPT_PIN 0
#define FRONT_RIGHT_INTERUPT_PIN 2
#define BACK_LEFT_INTERUPT_PIN 25
#define BACK_RIGHT_INTERUPT_PIN 3

const float WHEEL_SEP_LENGTH =
    .130;  // how far wheels are apart length meters
const float WHEEL_SEP_WIDTH =
    .092;  // how far wheels are apart width meters
const float WHEEL_RADIUS = .0315;  // radius of wheels meters
const float WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2 * M_PI;
const int ENCODER_TICKS_PER_SECOND = 20;

float convert_radians_per_sec_to_meters_per_sec(float radians_per_sec);

float convert_meters_per_sec_to_radians_per_sec(float meters_per_sec);

constexpr float MAX_PWM = 4000;
constexpr float MIN_PWM = 900;
constexpr float MAX_PID_CONTROL = 1;
constexpr float MIN_PID_CONTROL = 0;
constexpr float SLOPE_PID =
    (MAX_PWM - MIN_PWM) / (MAX_PID_CONTROL - MIN_PID_CONTROL);

}  // end namespace util
#endif