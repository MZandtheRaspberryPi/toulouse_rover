#include "util.h"

namespace util {

float convert_radians_per_sec_to_meters_per_sec(float radians_per_sec) {
  return radians_per_sec * WHEEL_CIRCUMFERENCE / (2 * M_PI);
}

float convert_meters_per_sec_to_radians_per_sec(float meters_per_sec) {
  return meters_per_sec * 2 * M_PI / WHEEL_CIRCUMFERENCE;
}

}  // end namespace util