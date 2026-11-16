#pragma once
#include "Arduino.h"
class BLEServer;
class BLECharacteristic;
class BLECharacteristicCallbacks {
 public:
  virtual ~BLECharacteristicCallbacks() {}
  virtual void onWrite(BLECharacteristic *c) {}
};
class BLEDescriptor { public: virtual ~BLEDescriptor() {} };
class BLE2902 : public BLEDescriptor {};
class BLECharacteristic {
 public:
  static const uint32_t PROPERTY_READ = 1, PROPERTY_WRITE = 2, PROPERTY_NOTIFY = 4, PROPERTY_WRITE_NR = 32;
  std::string uuid, value;
  BLECharacteristicCallbacks *cb = nullptr;
  std::vector<std::string> notified;
  void setCallbacks(BLECharacteristicCallbacks *c) { cb = c; }
  void addDescriptor(BLEDescriptor *) {}
  void setValue(const uint8_t *d, size_t n) { value.assign((const char *)d, n); }
  void notify(bool = true) { notified.push_back(value); }
  String getValue() const { return String(value); }
};
class BLEService {
 public:
  std::vector<BLECharacteristic *> chars;
  BLECharacteristic *createCharacteristic(const char *uuid, uint32_t) { auto c = new BLECharacteristic(); c->uuid = uuid; chars.push_back(c); return c; }
  void start() {}
};
class BLEServerCallbacks {
 public:
  virtual ~BLEServerCallbacks() {}
  virtual void onConnect(BLEServer *s) {}
  virtual void onDisconnect(BLEServer *s) {}
};
class BLEServer {
 public:
  BLEServerCallbacks *cb = nullptr;
  BLEService *svc = nullptr;
  uint16_t mtu = 23;
  int disconnects = 0;
  void setCallbacks(BLEServerCallbacks *c) { cb = c; }
  BLEService *createService(const char *) { svc = new BLEService(); return svc; }
  uint16_t getPeerMTU(uint16_t) { return mtu; }
  uint16_t getConnId() { return 0; }
  void disconnect(uint16_t) { disconnects++; }
};
class BLEAdvertising {
 public:
  void addServiceUUID(const char *) {}
  void setScanResponse(bool) {}
};
class BLEDevice {
 public:
  static BLEServer *srv;
  static int advStarts;
  static bool init(String) { return true; }
  static int setMTU(uint16_t) { return 0; }
  static BLEServer *createServer() { srv = new BLEServer(); return srv; }
  static BLEAdvertising *getAdvertising() { static BLEAdvertising a; return &a; }
  static void startAdvertising() { advStarts++; }
};
