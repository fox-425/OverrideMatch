#include "data/dataLogger.h"
#include "pros/rtos.hpp"
#include <bit>

#define LOG_FILE_PATH "/usd/log.bin"

Data::Data(uint32_t time, uint32_t RotY, uint32_t RotX, uint32_t RotC, double DI, double D1, double D2, double D3, double D4):
  pidCount(0),
  data({
    (uint64_t)time << 32 | (uint64_t)RotY,
    (uint64_t)RotX << 32 | (uint64_t)RotC,
    std::bit_cast<uint64_t>(DI),
    std::bit_cast<uint64_t>(D1),
    std::bit_cast<uint64_t>(D2),
    std::bit_cast<uint64_t>(D3),
    std::bit_cast<uint64_t>(D4)
  }),
  pidData{}
{}

void Data::addPID(uint32_t p, uint32_t i, uint32_t d) {
  if (pidCount == 8) {
    // send error to screenwriter
    return;
  }
  uint8_t sel = (uint8_t) ((double)pidCount*3.0/2.0);
  if (pidCount%2 == 0) {
    pidData[sel] = (uint64_t)p << 32;
    pidData[sel] |= (uint64_t)i;
    pidData[sel+1] = (uint64_t)d << 32;
  } else {
    pidData[sel] |= (uint64_t)p;
    pidData[sel+1] = (uint64_t)i << 32;
    pidData[sel+1] |= (uint64_t)d;
  }
  pidCount++;
}

Buffer::Buffer():
  bits{},
  index(0)
{}

void Buffer::addToBuffer(Data &data) {
  uint16_t select = (uint16_t) (index/64);
  uint8_t next = (uint8_t) (index%64);
  if (next != 0) {
    for (int i = 0; i < 7; i++) {
        bits[select+i] |= data.data[i] >> next;
        bits[select+i+1] = data.data[i] << 64-next;
    }
  } else {
    for (int i = 0; i < 7; i++) {
      bits[select+i] = data.data[i];
    }
  }
  index += 448;
  select = (uint16_t) (index/64);
  next = (uint8_t) (index%64);
  if (data.pidCount == 0) {
    index++;
  } else {
    int loops = (int) (((double)data.pidCount * 3.0+1)/2.0);
    if (next != 0) {
      for (int i = 0; i < loops; i++) {
        bits[select+i] |= data.pidData[i] >> next;
        bits[select+i+1] = data.pidData[i] << 64-next;
      }
    } else {
      for (int i = 0; i < loops; i++) {
        bits[select+i] = data.pidData[i];
      }
    }
    index += 96*data.pidCount;
  }
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
