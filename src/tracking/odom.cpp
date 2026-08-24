#include "tracking/odom.h"
#include <cmath>

Odom::Odom(double diaVert, double diaHori) {
  lastMillis = 0;
  lastCentiVert = 0;
  lastCentiHori = 0;
  multVert = diaVert*M_PI/36000;
  multHori = diaHori*M_PI/36000;
};

void Odom::update(uint32_t millis, double rads, int32_t centiVert, int32_t centiHori) {
  uint32_t dt = millis-lastMillis;
  double dRads = rads-lastRads;
  double dVert = (double)(centiVert-lastCentiVert) * multVert;
  double dHori = (double)(centiHori-lastCentiHori) * multHori;

  double dLocalX;
  double dLocalY;
  if (std::fabs(dRads) < 1e-6) {
    dLocalX = dHori;
    dLocalY = dVert;
  } else {
    double mult = 2.0 * sin(dRads * 0.5) / dRads;
    dLocalX = dHori * mult;
    dLocalY = dVert * mult;
  }

  double mag = sqrt(dLocalX*dLocalX + dLocalY*dLocalY);
  double rads2 = (fabs(dLocalX) < 1e-6 && fabs(dLocalY) < 1e-6) ? 0 : atan2(dLocalY, dLocalX) - rads - (dRads * 0.5);
  
  dx = cos(rads2) * mag;
  dy = sin(rads2) * mag;

  lastMillis = millis;
  lastRads = rads;
  lastCentiVert = centiVert;
  lastCentiHori = centiHori;
};