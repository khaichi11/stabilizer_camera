#pragma once
#include "Arduino.h"
class TwoWire {
 public:
  bool begin(int sda, int scl, uint32_t freq = 0);
  bool end();
  void setTimeOut(uint16_t);
  void beginTransmission(uint16_t addr);
  void beginTransmission(int addr) { beginTransmission((uint16_t)addr); }
  size_t write(uint8_t b);
  uint8_t endTransmission(bool sendStop = true);
  uint8_t requestFrom(int address, int size);
  int read();
};
extern TwoWire Wire;
