#pragma once
#include "Arduino.h"
#include <map>
class Preferences {
 public:
  std::map<std::string, std::vector<uint8_t>> m;
  bool begin(const char *, bool = false, const char * = nullptr) { return true; }
  size_t getBytesLength(const char *k) { auto it = m.find(k); return it == m.end() ? 0 : it->second.size(); }
  size_t getBytes(const char *k, void *b, size_t n) { auto it = m.find(k); if (it == m.end()) return 0; size_t c = std::min(n, it->second.size()); memcpy(b, it->second.data(), c); return c; }
  size_t putBytes(const char *k, const void *b, size_t n) { m[k] = std::vector<uint8_t>((const uint8_t *)b, (const uint8_t *)b + n); return n; }
  uint32_t getUInt(const char *k, uint32_t d = 0) { auto it = m.find(k); if (it == m.end()) return d; uint32_t v; memcpy(&v, it->second.data(), 4); return v; }
  size_t putUInt(const char *k, uint32_t v) { return putBytes(k, &v, 4); }
};
