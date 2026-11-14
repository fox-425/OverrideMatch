#pragma once
#include <cstdint>
#include "pros/rtos.hpp"

namespace Printing {
  inline double x = 0;
  inline double y = 0;
  inline double h = 0;
  inline uint8_t celM1 = 0;
  inline uint8_t celM2 = 0;
  inline uint8_t celM3 = 0;
  inline uint8_t celM4 = 0;
  inline int32_t mvM1 = 0;
  inline int32_t mvM2 = 0;
  inline int32_t mvM3 = 0;
  inline int32_t mvM4 = 0;
  inline uint8_t celMC1 = 0;
  inline uint8_t celMC2 = 0;
  inline uint8_t celMC3 = 0;
  inline int32_t mvMC1 = 0;
  inline int32_t mvMC2 = 0;
  inline int32_t mvMC3 = 0;
  inline int32_t amMC1 = 0;
  inline int32_t amMC2 = 0;
  inline int32_t amMC3 = 0;

  void init();

  void loop();

  void update();

  
}
