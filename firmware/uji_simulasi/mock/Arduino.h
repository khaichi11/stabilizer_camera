// host mock of the ESP32 Arduino core (only what the firmware uses)
#pragma once
#include <cstdint>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <cstddef>
#include <cstdarg>
#include <strings.h>
#include <string>
#include <deque>
#include <vector>
#define ESP_ARDUINO_VERSION_MAJOR 3
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
class String {
 public:
  std::string s;
  String() {}
  String(const char *c) : s(c ? c : "") {}
  String(const std::string &x) : s(x) {}
  size_t length() const { return s.size(); }
  char operator[](size_t i) const { return s[i]; }
  const char *c_str() const { return s.c_str(); }
  bool operator==(const String &o) const { return s == o.s; }
};
uint32_t millis();
uint32_t micros();
void delay(uint32_t ms);
void pinMode(int, int);
void digitalWrite(int, int);
bool ledcAttach(uint8_t pin, uint32_t freq, uint8_t res);
uint32_t ledcChangeFrequency(uint8_t pin, uint32_t freq, uint8_t res);
bool ledcWrite(uint8_t pin, uint32_t duty);
float temperatureRead();
struct MockSerial {
  std::deque<char> in;
  std::string out;
  bool connected = true;
  void begin(unsigned long) {}
  int available() { return (int)in.size(); }
  int read() { if (in.empty()) return -1; char c = in.front(); in.pop_front(); return (uint8_t)c; }
  int availableForWrite() { return 4096; }
  size_t write(uint8_t c) { out.push_back((char)c); return 1; }
  size_t write(const uint8_t *b, size_t n) { out.append((const char *)b, n); return n; }
  explicit operator bool() const { return connected; }
  void feed(const char *s) { while (*s) in.push_back(*s++); }
};
extern MockSerial Serial;
struct MockESP {
  uint32_t getFreeHeap() { return 123456; }
  void restart();
};
extern MockESP ESP;
// FreeRTOS queue
typedef std::deque<char> *QueueHandle_t;
#define pdTRUE 1
inline QueueHandle_t xQueueCreate(int, int) { return new std::deque<char>(); }
inline int xQueueSend(QueueHandle_t q, const void *p, int) { q->push_back(*(const char *)p); return 1; }
inline int xQueueReceive(QueueHandle_t q, void *p, int) { if (q->empty()) return 0; *(char *)p = q->front(); q->pop_front(); return 1; }
