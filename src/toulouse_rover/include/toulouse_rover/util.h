#ifndef util_h
#define util_h


#include <math.h>

namespace util {

const float WHEEL_SEP_LENGTH =
    .130;  // how far wheels are apart length meters
const float WHEEL_SEP_WIDTH =
    .092;  // how far wheels are apart width meters
const float WHEEL_RADIUS = .024;  // radius of wheels meters
const float WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2 * M_PI;

float convert_radians_per_sec_to_meters_per_sec(float radians_per_sec);

float convert_meters_per_sec_to_radians_per_sec(float meters_per_sec);

}  // end namespace util
#endif