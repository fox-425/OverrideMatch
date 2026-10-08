#include "data/dataLogger.h"
#include "data/EMSG.h"
#include <bit>

extern EMSG eMsg;
#define LOG_FILE_PATH "/usd/log.bin"

Data::Data(uint32_t time, uint32_t RotY, uint32_t RotX, uint32_t RotC, double DI, double D1, double D2, double D3, double D4, double head, double x, double y):
  time(time), RotY(RotY) RotX(RotX), RotC(RotC), DI(DI), D1(D1), D2(D2), D3(D3), D4(D4), head(head), x(x), y(y)
{}

PidData::PidData(uint32_t time, uint8_t id, float p, float i, float d):
  time(time), id(id), p(p), i(i), d(d)
{}

Buffer::Buffer():
  bits{},
  index(0)
{}

void Buffer::addToBuffer(Data &data) {
  arr[index] = data;
  index++;
}

void PidBuffer::addToBuffer(PidData &data) {
  arr[index] = data;
  index++;
}

void Buffer::clearBuffer() {
  index = 0;
  arr.fill(0);
}

void PidBuffer::clearBuffer() {
  index = 0;
  arr.fill(0);
}

Logger::Logger():
  bufferActive(true),
  B1(Buffer()),
  B2(Buffer()),
  PB1(PidBuffer()),
  PB2(PidBuffer())
{}

void Logger::switchBuffer() {
  bufferActive = !bufferActive;
}

void Logger::addToBuffer(Data &data) {
  if (bufferActive) {
    B1.mutex.lock();
    B1.addToBuffer(data);
    B1.mutex.unlock();
  } else {
    B2.mutex.lock();
    B2.addToBuffer(data);
    B2.mutex.unlock();
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
    eMsg.addText("Failed to open log file. ");
    return;
  }
  // while (false) {
  //   switchBuffer();
  //   if (buffer1Active) {buffer1.mutex.lock();}
  //   else {buffer2.mutex.lock();}
    
  //   std::array<uint64_t, 8192> data = buffer1Active ? buffer2.flushBuffer() : buffer1.flushBuffer();
  //   if (buffer1Active) {buffer1.mutex.unlock();}
  //   else {buffer2.mutex.unlock();}
    
  //   fwrite(data.data(), sizeof(uint64_t), 8192, file);
  //   fflush(file);
    
  //   pros::delay(100);
  // }

  Data testData1(0, 1, 2, 3, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8);
  addToBuffer(testData1);
  fwrite(B1.arr.data(), sizeof(Buffer), B1.index, file);
  B1.clearBuffer();

  PidData testData2(0, 1, 0.1, 0.2, 0.3);
  addToBuffer(testData2);
  fwrite(PB1.bits.data(), sizeof(PidBuffer), PB1.index, file);
  PB1.clearBuffer();

  fflush(file);
  fclose(file);
  eMsg.addText("Test Logging complete. ");
}
