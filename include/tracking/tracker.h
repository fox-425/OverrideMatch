#pragma once
#include "drive/drive.h"
#include "tracking/odom.h"
#include "pros/rotation.hpp"
#include "pros/imu.hpp"

class Tracker {
  struct TimeStamp {
    uint32_t millis;
    double rot;
    int32_t centiVert;
    int32_t centiHori;

    TimeStamp(uint32_t millis, double rot, int32_t centiVert, int32_t centiHori);
  };

  Drive& chassis;
  Odom& odom;

  Imu& imu;
  Rotation& rotVert;
  Rotation& rotHori;

  TimeStamp past;

  double x;
  double y;
  
public:
  Tracker(Drive& chassis, Odom& odom, Imu& imu, Rotation& rotVert, Rotation& rotHori);

  void update();

  void reset();

  double getX();
  double getY();
};