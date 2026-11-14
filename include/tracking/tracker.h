#pragma once
#include "drive/drive.h"
#include "tracking/odom.h"
#include "pros/rotation.hpp"
#include "pros/imu.hpp"

struct Tracker {
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
  
  Tracker(Drive& chassis, Odom& odom, Imu& imu, Rotation& rotVert, Rotation& rotHori);

  void updateInit();
  void updateLoop();
  void update();

  void reset();
  void setPos(double x, double y, double h);

  double getX();
  double getY();
  double getH();
};