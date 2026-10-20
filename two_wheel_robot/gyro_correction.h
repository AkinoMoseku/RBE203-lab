#ifndef ROBOT_GYRO_CORRECTION_H
#define ROBOT_GYRO_CORRECTION_H

#include <stdint.h>
#include <math.h>

inline int16_t gyroXForCourse(int16_t raw, float measuredBias)
{
  // Course Angletest subtracts 128.1 internally. Adapt its input to preserve
  // the measured zero bias without modifying the course library.
  const float adjusted = (float)raw - measuredBias + 128.1f;
  if (adjusted >= 32767.0f) return INT16_MAX;
  if (adjusted <= -32768.0f) return INT16_MIN;
  return (int16_t)roundf(adjusted);
}

#endif
