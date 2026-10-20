#ifndef ROBOT_PID_CONTROLLER_H
#define ROBOT_PID_CONTROLLER_H

#include <math.h>

// Parallel PID: Kp*error + integral(Ki*error*dt) + Kd*filtered_error_rate.
// Time is in seconds. Only I is limited here; the course motor mixer limits
// final wheel PWM. Limiting each whole PID would change cancellation in the mix.
class PidController
{
public:
  float pTerm = 0, iTerm = 0, dTerm = 0, output = 0;

  void reset()
  {
    pTerm = iTerm = dTerm = output = 0;
    previousError = filteredRate = previousIntegral = integralChange = 0;
    initialized = false;
  }

  float update(float error, float seconds, float kp, float ki, float kd,
               bool useMeasuredRate = false, float measuredErrorRate = 0)
  {
    if (!(seconds > 0) || !isfinite(seconds) || !isfinite(error) ||
        !isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
        (useMeasuredRate && !isfinite(measuredErrorRate)))
    {
      reset();
      return output;
    }
    const float rate = useMeasuredRate ? measuredErrorRate :
                       (initialized ? (error - previousError) / seconds : 0);
    const float alpha = seconds / (0.020f + seconds); // D low-pass, tau=20 ms.
    filteredRate = initialized ? filteredRate + alpha * (rate - filteredRate) : rate;
    previousError = error;
    initialized = true;

    // A disabled I term must not keep an old integral contribution.
    if (ki == 0) iTerm = 0;
    previousIntegral = iTerm;
    iTerm = limitIntegral(iTerm + ki * error * seconds);
    integralChange = iTerm - previousIntegral;
    pTerm = kp * error;
    dTerm = kd * filteredRate;
    output = pTerm + iTerm + dTerm;
    return output;
  }

  float integralDelta() const { return integralChange; }

  // Called only for a PID updated on this tick, if its new I contribution
  // drives either saturated wheel farther beyond the available PWM.
  void undoIntegral()
  {
    iTerm = previousIntegral;
    integralChange = 0;
    output = pTerm + iTerm + dTerm;
  }

private:
  float previousError = 0, filteredRate = 0;
  float previousIntegral = 0, integralChange = 0;
  bool initialized = false;

  static float limitIntegral(float value)
  {
    if (value > 255.0f) return 255.0f;
    if (value < -255.0f) return -255.0f;
    return value;
  }
};

inline bool pushesWheelFurtherIntoSaturation(float demand, float change)
{
  return (demand > 255.0f && change > 0) ||
         (demand < -255.0f && change < 0);
}

// A/speed I -> (-dI,-dI); yaw I -> (-dI,+dI).
inline bool integralWorsensWheelSaturation(float leftDemand, float rightDemand,
                                         float integralDelta, bool yaw)
{
  return pushesWheelFurtherIntoSaturation(leftDemand, -integralDelta) ||
         pushesWheelFurtherIntoSaturation(rightDemand, yaw ? integralDelta : -integralDelta);
}

#endif
