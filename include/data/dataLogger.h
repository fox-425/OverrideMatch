#pragma once
#include "pros/rtos.hpp"
#include <cstdint>
#include <array>

struct Data {
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
};

struct PidData {
  uint32_t time;
  uint8_t id;
  uint32_t p;
  uint32_t i;
  uint32_t d;
  PidData(uint32_t time, uint8_t id, uint32_t p, uint32_t i, uint32_t d);
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

struct Logger {
  bool bufferActive;
  Buffer B1;
  Buffer B2;
  PidBuffer PB1;
  PidBuffer PB2;
  Logger();

  void switchBuffer();
  void addToBuffer(Data &data);
  void addToBuffer(PidData &data);

  void startLogging();
  void logToSD();
};
