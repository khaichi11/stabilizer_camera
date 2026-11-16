// host mock of ESPmDNS
#pragma once
#include <string>
struct MDNSResponder {
  std::string host;
  int services = 0;
  bool begin(const char *h) { host = h; return true; }
  void addService(const char *, const char *, int) { services++; }
};
extern MDNSResponder MDNS;
