#pragma once
#include "pros/rtos.hpp"
#include <cstdint>
#include <array>

struct Data {
  uint8_t dataTag = 0x01;
  uint32_t time;
  uint32_t RotY;
  uint32_t RotX;
  uint32_t RotC;
  double DI;
  double D1;
  double D2;
  double D3;
  double D4;
  double head;
  double x;
  double y;
  Data(uint32_t time, uint32_t RotY, uint32_t RotX, uint32_t RotC, double DI, double D1, double D2, double D3, double D4, double head, double x, double y);
  Data();
};

struct PidData {
  uint8_t dataTag = 0x02;
  uint16_t id;
  uint32_t time;
  float p;
  float i;
  float d;
  float tar;
  PidData(uint32_t time, float p, float i, float d, float tar, uint16_t id);
  PidData();
};

struct AutonData {
  uint8_t dataTag = 0x03;
  uint8_t type;
  uint16_t id;
  uint32_t time;
  AutonData(uint32_t time, uint8_t type, uint16_t id);
  AutonData();
};

struct Buffer {
  std::array<Data, 128> arr;
  uint8_t index;
  pros::Mutex mutex;
  Buffer();

  void addToBuffer(Data &data);
  void clearBuffer();
};

struct PidBuffer {
  std::array<PidData, 128> arr;
  uint8_t index;
  pros::Mutex mutex;
  PidBuffer();

  void addToBuffer(PidData &data);
  void clearBuffer();
};

struct AutonBuffer {
  std::array<AutonData, 128> arr;
  uint8_t index;
  pros::Mutex mutex;
  AutonBuffer();

  void addToBuffer(AutonData &data);
  void clearBuffer();
};

struct Logger {
  bool bufferActive;
  Buffer B1;
  Buffer B2;
  PidBuffer PB1;
  PidBuffer PB2;
  AutonBuffer AB1;
  AutonBuffer AB2;
  Logger();

  void switchBuffer();
  void addToBuffer(Data &data);
  void addToBuffer(Data &&data);
  void addToBuffer(PidData &data);
  void addToBuffer(PidData &&data);
  void addToBuffer(AutonData &data);
  void addToBuffer(AutonData &&data);

  void startLogging();
  void logToSD();
};
