#include "data/dataLogger.h"
#include "pros/rtos.hpp"

#define LOG_FILE_PATH "/usd/log.bin"

Data::Data(uint32_t time, uint32_t RotY, uint32_t RotX, uint32_t RotC, double DI, double D1, double D2, double D3, double D4):
  pidCount(0)
{
  data[0] = (uint64_t)time << 32;
  data[0] |= (uint64_t)RotY;
  data[1] = (uint64_t)RotX << 32;
  data[1] |= (uint64_t)RotC;
  data[2] = (uint64_t)DI;
  data[3] = (uint64_t)D1;
  data[4] = (uint64_t)D2;
  data[5] = (uint64_t)D3;
  data[6] = (uint64_t)D4;
}

void Data::addPID(uint32_t p, uint32_t i, uint32_t d) {
  uint8_t sel = (uint8_t) ((double)pidCount*2.0/3.0);
  if (pidCount%2 == 0) {
    pidData[sel] = (uint64_t)p << 32;
    pidData[sel] |= (uint64_t)i;
    pidData[sel+1] = (uint64_t)d << 32;
  } else {
    pidData[sel] |= (uint64_t)p;
    pidData[sel+1] = (uint64_t)d << 32;
    pidData[sel+1] |= (uint64_t)i;
  }
  pidCount++;
}

Buffer::Buffer():
  bits({0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U}),
  index(0)
{}

void Buffer::addToBuffer(Data data) {
  uint16_t select = index/64;
  uint8_t next = index % 64;
  uint8_t length = 64*(7+data.pidCount);
  if (length + index > 32) {
    bits[select] |= (uint64_t)data.data[0] << next;
  }
  // bits[select] |= (uint64_t)data.data[0] << next;
}

std::array<uint32_t, 8192> Buffer::flushBuffer() {
  // std::array<uint32_t, 8000> flushedBits = bits;
  // index = 0;
  // bits.fill(0x0U);
  // return flushedBits;
}

Logger::Logger():
  buffer1Active(true),
  buffer1(Buffer()),
  buffer2(Buffer())
{}

void Logger::switchBuffer() {
  buffer1Active = !buffer1Active;
}

void Logger::addToBuffer(Data data) {
  if (buffer1Active) {
    buffer1.addToBuffer(data);
  } else {
    buffer2.addToBuffer(data);
  }
}

void Logger::startLogging() {
  pros::Task logging([this]() {
    this->logToSD();
  });
}

void Logger::logToSD() {
  FILE* file = fopen(LOG_FILE_PATH, "wb");
  if (file == nullptr) {
    // send msg to screen writer
    return;
  }
  while (true) {
    switchBuffer();
    // std::array<uint32_t, 8000> data = buffer1Active ? buffer2.flushBuffer() : buffer1.flushBuffer();
    // fwrite(data.data(), sizeof(uint32_t), 8000, file);
    fflush(file);
    
    pros::delay(100);
  }
}