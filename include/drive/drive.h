#pragma once
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"
#include "pros/imu.hpp"
#include "pros/optical.hpp"
#include "pros/distance.hpp"
#include "pid.h"
using namespace pros;

struct MVs{
  int32_t MV1, MV2, MV3, MV4;
  MVs(int32_t MV1 = 0, int32_t MV2 = 0, int32_t MV3 = 0, int32_t MV4 = 0);
};

struct Drive {
  Motor& M1;
  Motor& M2;
  Motor& M3;
  Motor& M4;
  Motor& MC1;
  Motor& MC2;
  Motor& MC3;
  Motor& MT;
  adi::DigitalOut& claw;
  bool clawV;
  Optical& opt;
  pros::Distance& distC;
  
  int32_t mvC1;
  int32_t mvC2;
  int32_t mvC3;

  Drive(
    Motor& M1, Motor& M2, Motor& M3, Motor& M4,
    Motor& MC1, Motor& MC2, Motor& MC3, Motor& MT,
    adi::DigitalOut& claw, Optical& opt, pros::Distance& distC
  );

  void millivolts(int32_t mv1, int32_t mv2, int32_t mv3, int32_t mv4);
  void toggle(int32_t mv);
  uint8_t getColor();

  void cascade(int32_t mv);
  void clawT(bool val);
  void clawT();

  void brake(motor_brake_mode_e_t type);

  void tare_position_all();
};
