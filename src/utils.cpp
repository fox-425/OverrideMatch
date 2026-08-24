#include "utils.h"

double utils::degToRad(double deg) {
  return deg*M_PI/180.0;
}

double utils::radToDeg(double rad) {
  return rad*180.0/M_PI;
}

int32_t utils::sign(double num) {
  return (num < 0) ? -1 : 1;
}