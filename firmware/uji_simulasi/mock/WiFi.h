// host mock of the ESP32 WiFi API: an imaginary phone hotspot and fake TCP connections that the
// test writes requests into and reads replies from
#pragma once
#include "Arduino.h"
#include <memory>
#include <string>
#include <vector>
#include <deque>
struct IPAddress {
  uint8_t b[4];
  IPAddress(uint8_t a = 0, uint8_t c = 0, uint8_t d = 0, uint8_t e = 0) : b{a, c, d, e} {}
  String toString() const { char t[16]; snprintf(t, sizeof(t), "%u.%u.%u.%u", b[0], b[1], b[2], b[3]); return String(t); }
};
struct MockSock {
  std::string in, out;   // in: browser -> gimbal, out: gimbal -> browser
  bool open = true;      // gimbal side still has it
  bool peerOpen = true;  // browser side still has it
  size_t room = 1 << 20; // how much the network takes right now (0 = full)
};
std::vector<std::shared_ptr<MockSock>> &mockSocks();
class WiFiClient {
 public:
  std::shared_ptr<MockSock> s;
  int id = -1;
  WiFiClient() {}
  explicit WiFiClient(std::shared_ptr<MockSock> p) : s(p) {
    mockSocks().push_back(p);
    id = (int)mockSocks().size() - 1;
  }
  explicit operator bool() const { return (bool)s; }
  int available() { return (s && s->open) ? (int)s->in.size() : 0; }
  int read() {
    if (!available()) return -1;
    int c = (uint8_t)s->in[0];
    s->in.erase(0, 1);
    return c;
  }
  uint8_t connected() { return s && s->open && s->peerOpen; }
  void stop() { if (s) s->open = false; }
  int fd() const { return id; }
  void setNoDelay(bool) {}
};
class WiFiServer {
 public:
  std::deque<WiFiClient> pending;
  explicit WiFiServer(int) {}
  void begin() {}
  void setNoDelay(bool) {}
  WiFiClient accept() {
    if (pending.empty()) return WiFiClient();
    WiFiClient c = pending.front();
    pending.pop_front();
    return c;
  }
};
enum { WIFI_OFF = 0, WIFI_STA = 1, WIFI_AP = 2, WIFI_AP_STA = 3 };
enum { WL_IDLE_STATUS = 0, WL_NO_SSID_AVAIL = 1, WL_CONNECTED = 3, WL_CONNECT_FAILED = 4, WL_DISCONNECTED = 6 };
enum { WIFI_POWER_8_5dBm = 34 };
enum { WIFI_AUTH_OPEN = 0, WIFI_AUTH_WPA2_PSK = 3 };
class WiFiClass {
 public:
  // the phone hotspot in the test
  bool hotspotOn = true;
  std::string hotspotSsid = "HP Khair", hotspotPass = "rahasia123";
  // what the firmware did
  int modeNow = WIFI_OFF, beginCount = 0, txPower = 0, apStations = 0;
  bool ap = false, sleep = true;
  std::string joinSsid, joinPass, apName, apPassword, host;
  uint32_t connectAt = 0, scanDoneAt = 0;
  bool mode(int m) { modeNow = m; if (!(m & WIFI_AP)) ap = false; return true; }
  void macAddress(uint8_t *m) { const uint8_t v[6] = {0x34, 0x85, 0x18, 0x01, 0xA1, 0xB2}; memcpy(m, v, 6); }
  void persistent(bool) {}
  bool setAutoReconnect(bool) { return true; }
  bool setHostname(const char *h) { host = h; return true; }
  bool setSleep(bool on) { sleep = on; return true; }
  bool setTxPower(int p) { txPower = p; return true; }
  int begin(const char *s, const char *p) { joinSsid = s; joinPass = p ? p : ""; beginCount++; connectAt = millis() + 1500; return 0; }
  bool disconnect(bool = false, bool = false) { joinSsid.clear(); return true; }
  int status() {
    if (!(modeNow & WIFI_STA) || joinSsid.empty() || !hotspotOn || joinSsid != hotspotSsid || joinPass != hotspotPass) return WL_DISCONNECTED;
    return (int32_t)(millis() - connectAt) >= 0 ? WL_CONNECTED : WL_DISCONNECTED;
  }
  bool softAP(const char *s, const char *p, int = 1, int = 0, int = 4) { ap = true; apName = s; apPassword = p; return true; }
  bool softAPdisconnect(bool) { ap = false; return true; }
  int softAPgetStationNum() { return ap ? apStations : 0; }
  IPAddress localIP() { return IPAddress(172, 20, 10, 3); }
  IPAddress softAPIP() { return IPAddress(192, 168, 4, 1); }
  int RSSI() { return -52; }
  int16_t scanNetworks(bool, bool, bool, uint32_t) { scanDoneAt = millis() + 2000; return -1; }
  int16_t scanComplete() { return (int32_t)(millis() - scanDoneAt) >= 0 ? 3 : -1; }
  String SSID(uint8_t i) { static const char *n[] = {"HP Khair", "Wifi \"Rumah\"", "HP Khair"}; return String(n[i % 3]); }
  int RSSI(uint8_t i) { return -45 - 10 * i; }
  int encryptionType(uint8_t i) { return i == 1 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK; }
  void scanDelete() {}
};
extern WiFiClass WiFi;
