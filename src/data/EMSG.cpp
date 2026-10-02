#include "data/EMSG.h"

EMSG::EMSG(): content("") {}

void EMSG::addText(std::string text) {
  mutex.lock();
  content += text;
  mutex.unlock();
}

void EMSG::clearText() {
  mutex.lock();
  content = "";
  mutex.unlock();
  }