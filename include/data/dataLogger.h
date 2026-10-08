#pragma once
#include "pros/rtos.hpp"
#include <cstdint>
#include <array>

struct Data {
  std::array<uint64_t, 7> data;
  Data(uint32_t time, uint32_t RotY, uint32_t RotX, uint32_t RotC, double DI, double D1, double D2, double D3, double D4);
};

struct PidData {
  uint8_t id;
  std::array<uint64_t, 12> pidData;
  PidData(uint8_t id, uint32_t p, uint32_t i, uint32_t d);
};

struct Buffer {
  std::array<uint64_t, 8192> bits;
  uint32_t index;
  pros::Mutex mutex;

  Buffer();

  void addToBuffer(Data &data);

  void clearBuffer();
};

struct PidBuffer {
  std::array<uint64_t, 1024> bits;
  uint32_t index;
  pros::Mutex mutex;

  PidBuffer();

  void addToBuffer(PidData &data);

  void clearBuffer();
};

struct Logger {
  bool buffer1Active = true;
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
