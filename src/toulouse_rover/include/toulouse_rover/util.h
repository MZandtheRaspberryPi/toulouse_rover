#ifndef util_h
#define util_h

#include <math.h>
#include <vector>
#include <string>
#include <ros/ros.h>

namespace util
{
#define MOTOR_ENABLE_PIN 22

#define FRONT_LEFT_INTERUPT_PIN 0
#define FRONT_RIGHT_INTERUPT_PIN 2
#define BACK_LEFT_INTERUPT_PIN 25
#define BACK_RIGHT_INTERUPT_PIN 3

extern const float WHEEL_SEP_LENGTH;  // how far wheels are apart length meters
extern const float WHEEL_SEP_WIDTH;   // how far wheels are apart width meters
extern const float WHEEL_RADIUS;      // radius of wheels meters
extern const float WHEEL_CIRCUMFERENCE;
extern const int ENCODER_TICKS_PER_ROTATION;

extern const float MAX_PWM;
extern const float MIN_PWM;
extern const float MAX_PID_CONTROL;
extern const float MIN_PID_CONTROL;
extern const float SLOPE_PID;

// MAX I saw, with wheels not on ground, was about 60 encoder ticks per second,
// so 3 rotations, so 6 PI. Wheels have radius of .0315, circumfrence 0.198
// so if 3 rotations a second, 0.198 * 3 meters per second, this is 0.594meters/sec
// second in x direction, so this is our max speed
// motors off the ground didnt really start moving until 110 pwm, and when they were doing 11pwm
// saw 3 or 4 encoder ticks per 0.1 seconds. This is 30 ticks per second, or 1.5 rotations,
// so 3 PI radians. or 0.198 circumfrence * 1.5 = 0.297 meters per second
// this min speed is wheels off the ground, on the ground gotta overcome friction and other stuff
// and will likely be a higher min speed.
extern const float MAX_WHEEL_SPEED;
extern const float MIN_WHEEL_SPEED;
extern const float SLOPE_WHEEL_SPEED;

enum WheelConfigurationType
{
  OMNI_WHEELS,
  SKID_STEERING,
  DIFFERENTIAL_DRIVE
};

bool validate_wheel_config_type(const std::string& wheel_config_type);

std::string get_valid_wheel_config_types();

WheelConfigurationType getWheelConfigEnumFromStr(const std::string& wheel_config_type);

bool getWheelConfigTypeFromNH(ros::NodeHandle& nh, WheelConfigurationType& wheel_config_type);

float convert_radians_per_sec_to_meters_per_sec(float radians_per_sec);

float convert_meters_per_sec_to_radians_per_sec(float meters_per_sec);

}  // end namespace util
#endif