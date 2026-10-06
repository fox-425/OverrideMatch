#pragma once
#include "drive/drive.h"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"

class Control {
public:
  Drive& chassis;
  Controller& primary;

  Control(Drive& chassis, Controller& primary);

  uint8_t Buttons(uint8_t next);
  void Analog();
};
