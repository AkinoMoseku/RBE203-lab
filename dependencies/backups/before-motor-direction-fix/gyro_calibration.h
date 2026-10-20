#ifndef ROBOT_GYRO_CALIBRATION_H
#define ROBOT_GYRO_CALIBRATION_H

#include <stdint.h>
#include <math.h>

// Startup-only statistics. No course library or controller gain is changed.
struct GyroCalibration
{
  int32_t sums[6];
  int16_t minimum[6], maximum[6];
  uint16_t count;

  GyroCalibration() : sums{}, minimum{}, maximum{}, count(0) {}

  void add(const int16_t sample[6])
  {
    for (uint8_t i = 0; i < 6; ++i)
    {
      sums[i] += sample[i];
      if (count == 0 || sample[i] < minimum[i]) minimum[i] = sample[i];
      if (count == 0 || sample[i] > maximum[i]) maximum[i] = sample[i];
    }
    ++count;
  }

  float mean(uint8_t axis) const
  {
    return count ? (float)sums[axis] / count : 0.0f;
  }

  bool stationaryUpright() const
  {
    if (count < 400) return false;
    for (uint8_t i = 0; i < 6; ++i)
    {
      // Reject visible movement; this cannot replace holding the chassis still.
      const int32_t limit = i < 3 ? 1200 : 262; // about 0.073 g / 2 deg/s
      if ((int32_t)maximum[i] - minimum[i] > limit) return false;
      if (i >= 3 && fabsf(mean(i)) > 1310.0f) return false;
    }
    const float x = mean(0), y = mean(1), z = mean(2);
    const float gravitySquared = (x * x + y * y + z * z) / (16384.0f * 16384.0f);
    // +/-2g range; require approximately 1g and wheels-down near upright.
    return gravitySquared >= 0.8f * 0.8f &&
           gravitySquared <= 1.2f * 1.2f && z > 12000.0f &&
           fabsf(atan2f(y, z) * 57.3f) < 30.0f &&
           fabsf(atan2f(x, z) * 57.3f) < 30.0f;
  }
};

inline int16_t gyroXForCourse(int16_t raw, float measuredBias)
{
  // Angletest subtracts 128.1 internally. Adapt its input so the result is
  // (raw - measuredBias) / 131, apart from <= 0.5 raw count of rounding.
  // Keep the original raw sample available for logging.
  const float adjusted = (float)raw - measuredBias + 128.1f;
  if (adjusted >= 32767.0f) return INT16_MAX;
  if (adjusted <= -32768.0f) return INT16_MIN;
  return (int16_t)roundf(adjusted);
}

#endif
