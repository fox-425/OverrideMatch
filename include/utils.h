#pragma once
#include <cstdint>
#include <cmath>

namespace utils {
  double degToRad(double deg);
  double radToDeg(double rad);

  /**
   * @brief Returns the sign of the given number. 0 is positive.
   * @param num The number to evaluate.
   * @return 1 if the number is positive or zero, -1 if the number is negative.
  */
  int32_t sign(double num);
}