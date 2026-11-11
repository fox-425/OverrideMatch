#pragma once
#include <cstdint>

class Odom {
  uint32_t lastMillis;
  double lastRads;
  int32_t lastCentiVert;
  int32_t lastCentiHori;

  double multVert;
  double multHori;
  double offV;
  double offH;
public:
  Odom(double diaVert, double diaHori, double offV, double offH);

  void update(uint32_t millis, double rads, int32_t centiVert, int32_t centiHori);

  double dx;
  double dy;
};