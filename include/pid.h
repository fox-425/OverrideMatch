#pragma once
#include "utils.h"
#include <cstdint>

class PID {
  float target;
  float kp;
  float ki;
  float kd;

  float iMax;
  float iRange;

  float dRange;

  float errorRange1;
  float errorRange2;
  uint32_t errorTime1;
  uint32_t errorTime2;

  float sumError;
  float prevError;
  float output;
  uint32_t rangeTime1;
  uint32_t rangeTime2;
  bool firstTime;
  bool atTarget;

public:
  struct PIDoutput {
    float p;
    float i;
    float d;
    float out;
    PIDoutput();
    PIDoutput(float p, float i, float d);
  };

  PID(float target, float kp, float ki, float kd);

  /**
   * @param dRange When derivative value <= dRange, the pid begins to check if the pid is inside the error ranges
   * @param errorRange1 Large error tolerance value. When error to target is within this range, the pid will start the large timer
   * @param errorRange2 Small error tolerance value. When error to target is within this range, the pid will start the small timer
   * @param errorTime1 Large error mimimun time. When error is within the large error range, the pid will exit once errorTime1 has passed in ms
   * @param errorTime2 Small error mimimun time. When error is within the small error range, the pid will exit once errorTime2 has passed in ms
  */
  void setExit(float dRange, float errorRange1, float errorRange2, uint32_t errorTime1, uint32_t errorTime2);

  /**
   * @param iMax The maximum integral value. When iMax = 0, the integral value will not be capped
   * @param iRange When d value escapes iRange, the integral value will reset to 0
   */
  void setIntegral(float iMax, float iRange);

  /**
   * @param arrive The value to set arrive to
   */
  void setArrive(bool arrive);

  /**
   * @param target The target value to set the pid to
  */
  void setTarget(float target);

  /**
   * @return A boolean of if the pid should exit or not
  */
  bool exit();

  /**
   * @return The target value of the pid
  */
  float getTarget();

  /**
   * @return Whether the pid exited from small or large range. 0 still active, 1 large, 2 small
  */
  uint8_t exitType();

  /**
   * @param time The current time in ms
   * @param input The input value for the pid
  */
  PIDoutput update(int32_t time, float input);
};