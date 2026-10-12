#include "data/dataLogger.h"
#include "data/EMSG.h"
#include <bit>

extern EMSG eMsg;
#define LOG_FILE_PATH "/usd/log.bin"

Data::Data(uint32_t time, uint32_t RotY, uint32_t RotX, uint32_t RotC, double DI, double D1, double D2, double D3, double D4, double head, double x, double y):
  time(time), RotY(RotY), RotX(RotX), RotC(RotC), DI(DI), D1(D1), D2(D2), D3(D3), D4(D4), head(head), x(x), y(y)
{}

PidData::PidData(uint32_t time, float p, float i, float d, float tar, uint16_t id):
  time(time), p(p), i(i), d(d), tar(tar), id(id)
{}

AutonData::AutonData(uint32_t time, uint8_t type, uint16_t id):
  time(time), type(type), id(id)
{}

Data::Data():
  time(0), RotY(0), RotX(0), RotC(0), DI(0.0), D1(0.0), D2(0.0), D3(0.0), D4(0.0), head(0.0), x(0.0), y(0.0)
{}

PidData::PidData():
  time(0), id(0), p(0.0), i(0.0), d(0.0)
{}

AutonData::AutonData():
  time(0), type(0), id(0)
{}

Buffer::Buffer():
  arr{},
  index(0)
{}

PidBuffer::PidBuffer():
  arr{},
  index(0)
{}

AutonBuffer::AutonBuffer():
  arr{},
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

void AutonBuffer::addToBuffer(AutonData &data) {
  arr[index] = data;
  index++;
}

void Buffer::clearBuffer() {
  index = 0;
  arr.fill(Data(0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
}

void PidBuffer::clearBuffer() {
  index = 0;
  arr.fill(PidData(0, 0, 0.0, 0.0, 0.0, 0));
}

void AutonBuffer::clearBuffer() {
  index = 0;
  arr.fill(AutonData(0, 0, 0));
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

void Logger::addToBuffer(PidData &data) {
  if (bufferActive) {
    PB1.mutex.lock();
    PB1.addToBuffer(data);
    PB1.mutex.unlock();
  } else {
    PB2.mutex.lock();
    PB2.addToBuffer(data);
    PB2.mutex.unlock();
  }
}

void Logger::addToBuffer(AutonData &data) {
  if (bufferActive) {
    AB1.mutex.lock();
    AB1.addToBuffer(data);
    AB1.mutex.unlock();
  } else {
    AB2.mutex.lock();
    AB2.addToBuffer(data);
    AB2.mutex.unlock();
  }
}

void Logger::addToBuffer(Data &&data) {addToBuffer(data);}
void Logger::addToBuffer(PidData &&data) {addToBuffer(data);}
void Logger::addToBuffer(AutonData &&data) {addToBuffer(data);}


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
  fwrite(B1.arr.data(), sizeof(Data), B1.index, file);
  B1.clearBuffer();

  AutonData testData2(0, 2, 1);
  addToBuffer(testData2);
  fwrite(AB1.arr.data(), sizeof(AutonData), AB1.index, file);
  AB1.clearBuffer();

  PidData testData3(0, 0.0, 0.0, 0.0, 0.0, 1);
  PidData testData4(10, 0.1, 0.0, 0.01, 0.0, 1);
  PidData testData5(20, 0.2, 0.0, 0.08, 0.0, 1);
  PidData testData6(30, 0.1, 0.0, 0.03, 0.0, 1);
  PidData testData7(40, 0.0, 0.0, 0.0, 0.0, 1);
  addToBuffer(testData3);
  addToBuffer(testData4);
  addToBuffer(testData5);
  addToBuffer(testData6);
  addToBuffer(testData7);
  fwrite(PB1.arr.data(), sizeof(PidData), PB1.index, file);
  PB1.clearBuffer();

  fflush(file);
  fclose(file);
  eMsg.addText("Test Logging complete. ");
}
