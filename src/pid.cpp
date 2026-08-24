#include "pid.h"

PID::PIDoutput::PIDoutput():
  p(0.0f), i(0.0f), d(0.0f), out(0.0f)
{}

PID::PIDoutput::PIDoutput(float p, float i, float d):
  p(p), i(i), d(d), out(p+i+d)
{}

PID::PID(float target, float kp, float ki, float kd):
  target(target), kp(kp), ki(ki), kd(kd),
  iMax(0.0), iRange(0.0),
  dRange(1.0),
  errorRange1(1.0), errorRange2(2.5),
  errorTime1(100), errorTime2(500),
  sumError(0.0), prevError(0.0), output(0.0),
  rangeTime1(0), rangeTime2(0),
  firstTime(true), atTarget(false)
{}

void PID::setExit(float dRange, float errorRange1, float errorRange2, uint32_t errorTime1, uint32_t errorTime2) {
  this->dRange = dRange;
  this->errorRange1 = errorRange1;
  this->errorRange2 = errorRange2;
  this->errorTime1 = errorTime1;
  this->errorTime2 = errorTime2;
}

void PID::setIntegral(float iMax, float iRange) {
  this->iMax = iMax;
  this->iRange = iRange;
}

void PID::setArrive(bool arrive) {
  this->atTarget = arrive;
}

void PID::setTarget(float target) {
  this->target = target;
}

bool PID::exit() {
  return atTarget;
}

float PID::getTarget() {
  return target;
}

uint8_t PID::exitType() {
  if (!atTarget) {
    return 0;
  }
  uint32_t eL = rangeTime1+errorTime1;
  uint32_t eS = rangeTime2+errorTime2;

  if (eL < eS) {
    return 1;
  } else {
    return 2;
  }
}

PID::PIDoutput PID::update(int32_t time, float input) {
  float error = target - input;
  if (firstTime) {
    prevError = error;
    rangeTime1 = time;
    rangeTime2 = time;

    firstTime = false;
  }

  float p = kp * error;
  float d = kd * (error-prevError);

  float abs_error = std::fabs(error);
  int32_t sign_sumError = utils::sign(sumError);
  if (iRange != 0 && abs_error >= iRange) {
    sumError = 0.0f;
  } else {
    sumError += error;
    if (iMax != 0 && fabs(sumError)*ki > iMax) {
      sumError = (float)sign_sumError * iMax/ki;
    }
  }

  if (utils::sign(error) != sign_sumError || abs_error <= errorRange1) {
    sumError = 0.0f;
  }

  float i = ki * sumError;

  bool inDRange = (std::fabs(d) <= dRange);
  if (abs_error <= errorRange1) {
    if (inDRange && time-rangeTime1 >= errorTime1) {
      atTarget = true;
    }
  } else {
    rangeTime1 = time;
  }

  if (abs_error <= errorRange2) {
    if (inDRange && time-rangeTime2 >= errorTime2) {
      atTarget = true;
    }
  } else {
    rangeTime2 = time;
  }

  PIDoutput output(p, i, d);
  prevError = error;
  return output;
}