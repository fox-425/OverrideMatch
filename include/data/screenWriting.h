#pragma once
#include <cstdint>
#include "pros/rtos.hpp"

namespace Printing {
  inline uint8_t celM1 = 0;
  inline uint8_t celM2 = 0;
  inline uint8_t celM3 = 0;
  inline uint8_t celM4 = 0;
  inline uint8_t celM5 = 0;
  inline uint8_t celM6 = 0;
  inline uint8_t celMI = 0;
  inline uint8_t celMC1 = 0;
  inline uint8_t celMC2 = 0;

  void init();

  void loop();

  void update();

  
}
