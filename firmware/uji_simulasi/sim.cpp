// Host simulation of the gimbal: runs the real firmware (.ino) against a physics model of
// the two servos, the camera, an MPU6050 mounted upside-down and rotated, body motion while walking.
#include "Arduino.h"
#include "Wire.h"
#include "Preferences.h"
#include "WiFi.h"
#include "ESPmDNS.h"
#include "lwip/sockets.h"
#include <cerrno>
#include "esp_system.h"
#include "esp_random.h"
#include <random>
#include <algorithm>
#include <functional>
#include <map>

// ------------------------------------------------------------------ sim clock + physics state
static uint64_t t_us = 0;
static std::mt19937 rng(12345);
static std::normal_distribution<double> nd(0.0, 1.0);
MockSerial Serial;
MockESP ESP;
TwoWire Wire;
WiFiClass WiFi;
MDNSResponder MDNS;
std::vector<std::shared_ptr<MockSock>> &mockSocks() {
  static std::vector<std::shared_ptr<MockSock>> v;
  return v;
}
// non-blocking send onto a fake connection: takes what 'room' allows, EAGAIN when it is full
int send(int fd, const void *buf, size_t n, int) {
  auto &v = mockSocks();
  if (fd < 0 || fd >= (int)v.size() || !v[fd]->peerOpen) {
    errno = ECONNRESET;
    return -1;
  }
  MockSock &s = *v[fd];
  size_t k = std::min(n, s.room);
  if (!k) {
    errno = EAGAIN;
    return -1;
  }
  s.out.append((const char *)buf, k);
  return (int)k;
}
static bool restarted = false;
void MockESP::restart() { restarted = true; }

typedef double M3[3][3];
static void mmul(const M3 a, const M3 b, M3 o) {
  M3 t;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      t[i][j] = 0;
      for (int k = 0; k < 3; k++) t[i][j] += a[i][k] * b[k][j];
    }
  memcpy(o, t, sizeof(M3));
}
static void rx(double a, M3 o) { double c = cos(a), s = sin(a); M3 r = {{1, 0, 0}, {0, c, -s}, {0, s, c}}; memcpy(o, r, sizeof(M3)); }
static void ry(double a, M3 o) { double c = cos(a), s = sin(a); M3 r = {{c, 0, s}, {0, 1, 0}, {-s, 0, c}}; memcpy(o, r, sizeof(M3)); }
static void rz(double a, M3 o) { double c = cos(a), s = sin(a); M3 r = {{c, -s, 0}, {s, c, 0}, {0, 0, 1}}; memcpy(o, r, sizeof(M3)); }
static const double D2R = M_PI / 180.0;
static uint8_t regs[128];  // MPU6050 registers written by the firmware
static uint32_t ledcHz = 50;  // servo pulses per second, as the firmware set the LEDC

static double envd(const char *k, double d) { const char *v = getenv(k); return v ? atof(v) : d; }
// The servo bought for this gimbal: "MG90S Micro Servo compatible" (brand as in stock), 0.11 s/60 deg at
// 4.8 V (545 deg/s), 2.2 kg.cm, dead band width 10 us. Analog: it only reads a new pulse once per frame
// (the LEDC duty takes effect at the next period and the servo measures the pulse while it arrives), does
// nothing while the pulse is within half the dead band of its own position, follows with a lag and has a
// top speed. Change with SRV_DB (us, full width), SRV_TAU (ms), SRV_VMAX (deg/s), BACKLASH (deg),
// SRV_LATCH=0 (servo sees every write at once).
struct Servo {
  double ang = 0;      // deg, output shaft (after backlash)
  double motor = 0;    // deg, motor side
  double pending = 0;  // us, last value written to the LEDC
  double pulse = 0;    // us, what the servo works from (0 = no pulses)
  double usPerDeg = 11.0;
  void step(double dt) {
    if (pulse <= 0) return;
    double tgt = (pulse - 1500.0) / usPerDeg;
    double err = tgt - motor;
    if (fabs(err) > 0.5 * envd("SRV_DB", 10.0) / usPerDeg) {  // dead band
      double v = err / (envd("SRV_TAU", 25.0) * 1e-3);   // lag
      double vmax = envd("SRV_VMAX", 545.0);
      v = std::max(-vmax, std::min(vmax, v));
      motor += v * dt;
    }
    double bl = envd("BACKLASH", 0.0);
    if (ang < motor - bl / 2) ang = motor - bl / 2;
    if (ang > motor + bl / 2) ang = motor + bl / 2;
  }
};
// --- scenario parameters
struct World {
  Servo pitchS, rollS;
  bool swapped = true;         // pin3 drives the ROLL servo (cables swapped on purpose)
  double sP = -1, sR = +1;     // physical direction of each servo
  double alpha0 = 37.0, beta0 = -1.5;  // horn offsets: pitch at 1500 us = 37 deg, roll = -1.5 deg
  // body
  double leanF = 0, leanS = 0, yaw = 0;
  std::function<void(double, World &)> body;  // sets lean/yaw/linear acc per time
  double accLin[3] = {0, 0, 0};             // g, world
  // IMU mount: columns = IMU axes in camera coordinates (camera: X right, Y forward, Z up)
  M3 mount = {{0, -1, 0}, {-1, 0, 0}, {0, 0, -1}};  // chip down: imu x = -Y(cam), y = -X(cam), z = -Z(cam)
  double misF = 1.5, misL = 1.0;                     // deg, IMU not perfectly flat
  double gyroBias[3] = {1.2, -0.8, 0.5};
  bool imuConnected = true;
  M3 Rimu_prev;
  bool havePrev = false;
  double omega[3] = {0, 0, 0};  // deg/s in IMU frame
  double omega1[3] = {0, 0, 0};
  M3 Rcam, Rimu;
} W;

static void computeOrientation() {
  M3 Rb, a, b, c, t;
  rz(W.yaw * D2R, a);
  rx(W.leanF * D2R, b);
  ry(W.leanS * D2R, c);
  mmul(a, b, t);
  mmul(t, c, Rb);
  double alpha = W.alpha0 + W.sP * W.pitchS.ang;
  double beta = W.beta0 + W.sR * W.rollS.ang;
  rx(-alpha * D2R, a);
  ry(beta * D2R, b);
  mmul(Rb, a, t);
  mmul(t, b, W.Rcam);
  M3 mis, m1, m2;
  ry(W.misF * D2R, m1);  // about camera forward (Y)
  rx(W.misL * D2R, m2);
  mmul(m1, m2, mis);
  mmul(W.Rcam, mis, t);
  mmul(t, W.mount, W.Rimu);
}
static void truth(double &pitch, double &roll) {
  double Fz = W.Rcam[2][1], Lz = -W.Rcam[2][0], Uz = W.Rcam[2][2];
  pitch = asin(-Fz) / D2R;
  roll = atan2(Lz, Uz) / D2R;
}
static void physStep(double dt) {
  double t = t_us * 1e-6;
  if (W.body) W.body(t, W);
  // a pulse every 1/ledcHz s; the servo has measured it ~1.5 ms after it starts
  static int64_t lastFrame = -1;
  int64_t frame = ((int64_t)t_us - 1500) * ledcHz / 1000000;
  if (envd("SRV_LATCH", 1) == 0 || frame != lastFrame) {
    lastFrame = frame;
    W.pitchS.pulse = W.pitchS.pending;
    W.rollS.pulse = W.rollS.pending;
  }
  W.pitchS.step(dt);
  W.rollS.step(dt);
  computeOrientation();
  if (W.havePrev) {  // omega from R_prev^T R_now (IMU frame)
    M3 rt, S;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) rt[i][j] = W.Rimu_prev[j][i];
    mmul(rt, W.Rimu, S);
    double om[3] = {(S[2][1] - S[1][2]) / (2 * dt) / D2R, (S[0][2] - S[2][0]) / (2 * dt) / D2R, (S[1][0] - S[0][1]) / (2 * dt) / D2R};
    // MPU6050 digital low-pass filter, from the DLPF_CFG register (datasheet delay per setting), modelled as
    // two first-order stages with the same total delay
    static const double dlpfDelayMs[8] = {0.98, 1.9, 2.8, 4.8, 8.3, 13.4, 18.6, 0.98};
    double k = dt / (dlpfDelayMs[regs[0x1A] & 7] * 0.5e-3 + dt);
    for (int i = 0; i < 3; i++) {
      W.omega1[i] += (om[i] - W.omega1[i]) * k;
      W.omega[i] += (W.omega1[i] - W.omega[i]) * k;
    }
  }
  memcpy(W.Rimu_prev, W.Rimu, sizeof(M3));
  W.havePrev = true;
  t_us += (uint64_t)(dt * 1e6 + 0.5);
}
// ------------------------------------------------------------------ Arduino mock functions
uint32_t millis() { return (uint32_t)(t_us / 1000); }
uint32_t micros() { return (uint32_t)t_us; }
void delay(uint32_t ms) { for (uint32_t i = 0; i < ms * 4; i++) physStep(0.00025); }
void pinMode(int, int) {}
static int ledState = 1;
void digitalWrite(int pin, int v) { if (pin == 8) ledState = v; }
bool ledcAttach(uint8_t, uint32_t f, uint8_t) { ledcHz = f; return true; }
uint32_t ledcChangeFrequency(uint8_t, uint32_t f, uint8_t) { ledcHz = f; return f; }
static std::map<int, double> pinPulse;
bool ledcWrite(uint8_t pin, uint32_t duty) {
  double us = duty * 1e6 / ledcHz / 16383.0;
  pinPulse[pin] = us;
  Servo &s3 = W.swapped ? W.rollS : W.pitchS;
  Servo &s4 = W.swapped ? W.pitchS : W.rollS;
  if (pin == 3) s3.pending = us;
  if (pin == 4) s4.pending = us;
  return true;
}
float temperatureRead() { return 41.5f; }
esp_reset_reason_t esp_reset_reason() { return ESP_RST_POWERON; }
uint32_t esp_random() { return rng(); }
// ------------------------------------------------------------------ Wire / MPU6050 register model
static uint16_t wAddr = 0;
static uint8_t regPtr = 0;
static std::vector<uint8_t> wBuf, rBuf;
static size_t rPos = 0;
bool TwoWire::begin(int, int, uint32_t) { return true; }
bool TwoWire::end() { return true; }
void TwoWire::setTimeOut(uint16_t) {}
void TwoWire::beginTransmission(uint16_t a) { wAddr = a; wBuf.clear(); }
size_t TwoWire::write(uint8_t b) { wBuf.push_back(b); return 1; }
uint8_t TwoWire::endTransmission(bool) {
  if (!W.imuConnected || wAddr != 0x68) return 2;
  if (!wBuf.empty()) {
    regPtr = wBuf[0];
    for (size_t i = 1; i < wBuf.size(); i++) regs[(regPtr + i - 1) & 127] = wBuf[i];
  }
  return 0;
}
static void put16(std::vector<uint8_t> &v, double x) {
  long r = lround(x);
  r = std::max(-32768L, std::min(32767L, r));
  int16_t s = (int16_t)r;
  v.push_back((uint8_t)((uint16_t)s >> 8));
  v.push_back((uint8_t)(s & 0xFF));
}
uint8_t TwoWire::requestFrom(int address, int size) {
  rBuf.clear();
  rPos = 0;
  if (!W.imuConnected || address != 0x68) return 0;
  if (regPtr == 0x75) {
    rBuf.push_back(0x68);
  } else if (regPtr == 0x3B) {
    double up[3] = {0, 0, 1};
    double fw[3];
    for (int i = 0; i < 3; i++) fw[i] = up[i] + W.accLin[i];
    for (int k = 0; k < 3; k++) {  // specific force in IMU frame = Rimu^T * f
      double v = W.Rimu[0][k] * fw[0] + W.Rimu[1][k] * fw[1] + W.Rimu[2][k] * fw[2];
      put16(rBuf, (v + 0.004 * nd(rng)) * (16384 >> ((regs[0x1C] >> 3) & 3)));  // LSB per g from ACCEL_CONFIG
    }
    put16(rBuf, 0);  // temperature
    for (int k = 0; k < 3; k++) put16(rBuf, (W.omega[k] + W.gyroBias[k] + 0.05 * nd(rng)) * 32.8);
  }
  while ((int)rBuf.size() < size) rBuf.push_back(0);
  return (uint8_t)size;
}
int TwoWire::read() { return rPos < rBuf.size() ? rBuf[rPos++] : -1; }

// ------------------------------------------------------------------ the firmware
#include "../D435_gimbal_ESP32C3/D435_gimbal_ESP32C3.ino"

// ------------------------------------------------------------------ harness helpers
static std::string usbTaken;
static std::string takeUsb() { std::string s = Serial.out; Serial.out.clear(); usbTaken += s; return s; }
static void run(double sec, std::function<void()> each = nullptr) {
  uint64_t end = t_us + (uint64_t)(sec * 1e6);
  while (t_us < end) {
    loop();
    physStep(0.00025);
    if (each) each();
    if (restarted) return;
  }
}
static void usb(const char *cmd) { Serial.feed(cmd); Serial.feed("\n"); }
static std::vector<std::string> lines(const std::string &s) {
  std::vector<std::string> v;
  size_t a = 0;
  while (a < s.size()) {
    size_t b = s.find('\n', a);
    if (b == std::string::npos) b = s.size();
    v.push_back(s.substr(a, b - a));
    a = b + 1;
  }
  return v;
}
static void show(const std::string &s, const char *filt = nullptr, int maxl = 40) {
  int n = 0;
  for (auto &l : lines(s)) {
    if (l.empty() || l[0] == 'T') continue;
    if (filt && l.find(filt) == std::string::npos) continue;
    printf("   | %s\n", l.c_str());
    if (++n >= maxl) break;
  }
}
static int fails = 0;
static void check(bool ok, const char *what) {
  printf("  [%s] %s\n", ok ? "OK" : "FAIL", what);
  if (!ok) fails++;
}
// error statistics of the TRUE camera angle
struct Stat { double sum = 0, sum2 = 0, mx = 0; long n = 0; void add(double e) { sum += e; sum2 += e * e; mx = std::max(mx, fabs(e)); n++; } double rms() const { return n ? sqrt(sum2 / n) : 0; } double mean() const { return n ? sum / n : 0; } };

// ------------------------------------------------------------------ a browser on the phone/iPad
static std::shared_ptr<MockSock> tcpOpen(const std::string &request) {
  auto s = std::make_shared<MockSock>();
  s->in = request;
  webServer.pending.push_back(WiFiClient(s));
  return s;
}
static std::string httpGet(const std::string &path, const std::string &extra = "") {
  auto s = tcpOpen("GET " + path + " HTTP/1.1\r\nHost: gimbal-d435.local\r\nAccept-Encoding: gzip\r\n" + extra + "\r\n");
  for (int i = 0; i < 600 && s->open; i++) run(0.005);  // until the gimbal closes it
  return s->out;
}
static std::string hexs(const std::string &x) {
  static const char *H = "0123456789abcdef";
  std::string o;
  for (unsigned char c : x) { o += H[c >> 4]; o += H[c & 15]; }
  return o;
}
static std::shared_ptr<MockSock> wsOpen(const std::string &key = "dGhlIHNhbXBsZSBub25jZQ==") {
  auto s = tcpOpen("GET /ws HTTP/1.1\r\nHost: 172.20.10.3\r\nupgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: " + key +
                   "\r\nSec-WebSocket-Version: 13\r\nSec-WebSocket-Extensions: permessage-deflate\r\n\r\n");
  run(0.05);
  return s;
}
static void wsSend(std::shared_ptr<MockSock> s, const std::string &text) {  // a masked text frame, like a browser
  std::string f;
  f += (char)0x81;
  size_t n = text.size();
  if (n < 126) f += (char)(0x80 | n);
  else { f += (char)(0x80 | 126); f += (char)(n >> 8); f += (char)(n & 255); }
  const uint8_t mk[4] = {0x12, 0x34, 0x56, 0x78};
  f.append((const char *)mk, 4);
  for (size_t i = 0; i < n; i++) f += (char)(text[i] ^ mk[i & 3]);
  s->in += f;
}
static std::string wsTake(std::shared_ptr<MockSock> s) {  // the text of every whole frame received so far
  std::string &o = s->out, txt;
  size_t p = 0;
  if (o.rfind("HTTP/1.1 101", 0) == 0) p = o.find("\r\n\r\n") + 4;
  while (o.size() - p >= 2) {
    uint8_t op = o[p] & 0x0F;
    size_t len = (uint8_t)o[p + 1] & 0x7F, hl = 2;
    if (len == 126) {
      if (o.size() - p < 4) break;
      len = (size_t)(uint8_t)o[p + 2] << 8 | (uint8_t)o[p + 3];
      hl = 4;
    }
    if (o.size() - p < hl + len) break;
    if (op == 1) txt += o.substr(p + hl, len);
    p += hl + len;
  }
  o.erase(0, p);
  return txt;
}
static std::string nonceOf(const std::string &b, const char *after) {
  size_t p = b.find(after);
  return p == std::string::npos ? "" : b.substr(p + strlen(after), 32);
}
static void wsAuth(std::shared_ptr<MockSock> s, const char *pin) {
  wsSend(s, "HELLO");
  run(0.2);
  std::string n = nonceOf(wsTake(s), "\"nonce\":\"");
  char h[65];
  authHash(n.c_str(), pin, h);
  wsSend(s, std::string("AUTH ") + h);
  run(0.2);
}
static int countT(const std::string &b) {
  int n = 0;
  for (auto &l : lines(b))
    if (l.rfind("T,", 0) == 0) n++;
  return n;
}

int main() {
  printf("== SHA-256 self test\n");
  {
    Sha256 s;
    char out[65];
    shaInit(s);
    shaUpdate(s, (const uint8_t *)"abc", 3);
    shaFinalHex(s, out);
    check(!strcmp(out, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"), "sha256(abc)");
    std::string m(1000, 'a');
    shaInit(s);
    shaUpdate(s, (const uint8_t *)m.data(), m.size());
    shaFinalHex(s, out);
    printf("   sha256(1000*a) = %s\n", out);
    authHash("0123456789abcdef0123456789abcdef", "123456", out);
    printf("   authHash(nonce, 123456) = %s\n", out);
  }

  printf("== boot\n");
  W.body = nullptr;
  computeOrientation();
  setup();
  run(3.0);
  std::string o = takeUsb();
  show(o);
  check(imuOK, "IMU detected");
  check(fabs(pinPulse[3] - 1500) < 2 && fabs(pinPulse[4] - 1500) < 2 && mode == MODE_CENTER, "uncalibrated: servos held at centre");

  printf("== HELLO / GET / STATUS over USB\n");
  usb("HELLO");
  usb("GET");
  usb("STATUS");
  run(0.3);
  o = takeUsb();
  show(o);
  check(o.find("HELLO {") != std::string::npos && o.find("CFG {") != std::string::npos && o.find("ST {") != std::string::npos, "replies");

  printf("== CAL auto (cables swapped on purpose, IMU chip-down, pitch servo reversed)\n");
  usb("CAL auto");
  run(24.0);
  o = takeUsb();
  show(o, "CAL");
  show(o, "EV");
  check(o.find("CAL DONE auto") != std::string::npos, "auto calibration finished");
  char ax[8];
  axesString(ax, cfg.axF, cfg.sgF, cfg.axL, cfg.sgL, cfg.axU, cfg.sgU);
  check(!strcmp(ax, "-X+Y-Z"), "IMU axes detected as -X+Y-Z");
  check(cfg.swapServo == 1, "swapped servo cables detected");
  check(cfg.dirP == -1 && cfg.dirR == 1, "servo directions");
  printf("   usdeg p %.2f (true %.2f)  r %.2f (true %.2f)\n", cfg.usDegP, W.pitchS.usPerDeg, cfg.usDegR, W.rollS.usPerDeg);
  check(fabs(cfg.usDegP - W.pitchS.usPerDeg) < 0.5 && fabs(cfg.usDegR - W.rollS.usPerDeg) < 0.5, "servo scale within 0.5 us/deg");
  printf("   dead band measured: %.1f us (true %.1f, plus gear play)\n", cfg.srvDb, envd("SRV_DB", 10.0));
  if (!getenv("BACKLASH")) check(fabs(cfg.srvDb - envd("SRV_DB", 10.0)) < 2.5 && fabs(cfg.dbComp - 0.25f * cfg.srvDb) < 0.01, "servo dead band measured within 2.5 us, compensation set from it");
  printf("   servo timing measured: delay %.1f ms, lag %.1f ms (%d/8 steps)\n", cfg.srvDelay, cfg.srvTau, cal.tCount);
  check(cal.tCount >= 6 && cfg.srvDelay >= 0 && cfg.srvDelay < 40 && cfg.srvTau > 8 && cfg.srvTau < 2.5 * envd("SRV_TAU", 25.0), "servo delay and lag measured");
  printf("   bias %.2f %.2f %.2f (true %.2f %.2f %.2f)\n", cfg.bias[0], cfg.bias[1], cfg.bias[2], W.gyroBias[0], W.gyroBias[1], W.gyroBias[2]);

  // The default target is 0 (camera straight ahead, like a handheld stabiliser). The tests below model
  // belt use, camera 35 deg down, so they ask for it the way a user would.
  usb("SET pitch 35");
  printf("== stabilise, body still (IMU mounted 1.5/1.0 deg crooked, not yet levelled)\n");
  run(6.0);
  double tp, tr;
  truth(tp, tr);
  printf("   true camera: pitch %.2f roll %.2f   | measured: pitch %.2f roll %.2f\n", tp, tr, pitchMeas, rollMeas);
  check(fabs(pitchMeas - 35) < 0.4 && fabs(rollMeas) < 0.4, "measured angles reach the target");

  printf("== CAL level: user reads a phone level on top of the camera\n");
  {
    char cmd[64];
    snprintf(cmd, sizeof cmd, "CAL level %.1f %.1f", tp, tr);
    usb(cmd);
  }
  run(2.5);
  show(takeUsb(), "CAL");
  {
    const char *keys[] = {"kp", "gain", "ff", "smooth", "db", "acctau", "ffb", "pred", "sdelay", "stau", "sdb", "dbcomp", "acctauw", "dlpf", "pwmhz", "gait", "soft"};
    for (auto k : keys) {
      std::string env = k; for (auto &c : env) c = toupper(c);
      if (getenv(env.c_str())) { std::string c = std::string("SET ") + k + " " + getenv(env.c_str()); usb(c.c_str()); }
    }
  }
  run(5.0);
  truth(tp, tr);
  printf("   true camera after levelling: pitch %.2f roll %.2f\n", tp, tr);
  check(fabs(tp - 35) < 0.25 && fabs(tr) < 0.25, "camera truly at 35.0 / 0.0 within 0.25 deg");

  // two walking cases, each measured stabilised and then with the servos held in the centre
  //  goyang: the original test, lean 10/8 deg + 3 deg sway at 1.8 Hz, 0.3 g bumps
  //  jalan:  realistic gait at the belt: left-right at the stride (0.9 Hz), nod at every step (1.8 Hz),
  //          heel-strike jolt at each step, hip turning, forward/side accelerations, a little randomness
  auto goyang = [](double t, World &w) {
    double ramp = std::min(1.0, std::max(0.0, (t - 30.0) / 2.0));
    w.leanS = 10 * ramp + 3 * sin(2 * M_PI * 1.8 * t);
    w.leanF = 8 * ramp + 2.5 * sin(2 * M_PI * 1.8 * t + 1.0);
    w.yaw = 6 * sin(2 * M_PI * 0.9 * t);
    w.accLin[0] = 0;
    w.accLin[2] = 0.3 * sin(2 * M_PI * 3.6 * t);
    w.accLin[1] = 0.12 * sin(2 * M_PI * 1.8 * t + 0.4);
  };
  // Walking at the belt. Step timing varies a little from step to step like real walking (GAIT_JIT, default
  // 2 % step-rate variation), so a learned step rhythm is never exact.
  //  jalan:  left-right at the stride (0.9 Hz), nod at every step (1.8 Hz), heel-strike jolt at each step,
  //          hip turning, forward/side accelerations, a little random wobble
  //  tangga: ordinary walking, then up stairs 2 s into the measurement: slower stride, each step lifts the
  //          leg high and the right leg more than the left (bigger nod and heel strike on that side), wider
  //          left-right sway
  static const double WARM = envd("WARM", 10.0);  // s of walking before measuring (the rhythm is learned)
  struct Gaitgen {  // step clock with step-to-step variation
    double lastT = -1, cyc = 0, fj = 0;
    double step(double t, double f) {  // f = steps per second; returns steps taken
      double d = (lastT < 0 || t < lastT) ? 0 : t - lastT;
      if (lastT < 0 || t < lastT) cyc = fj = 0;
      lastT = t;
      static const double jit = envd("GAIT_JIT", 0.02);
      fj += -fj * d / 0.6 + jit * sqrt(2 * d / 0.6) * nd(rng);
      cyc += f * (1 + fj) * d;
      return cyc;
    }
  };
  // ~0.2 deg of random wobble. Filtered twice so it is smooth like a body (filtered once, its rate would
  // be white noise and look like violent vibration)
  struct Wobble {
    double f1 = 0, s1 = 0, f = 0, s = 0;
    void step() {
      f1 += (7.0 * nd(rng) - f1) * 0.005;
      s1 += (7.0 * nd(rng) - s1) * 0.005;
      f += (f1 - f) * 0.005;
      s += (s1 - s) * 0.005;
    }
  };
  auto jalan = [](double t, World &w) {
    static Gaitgen gg;
    static Wobble wb;
    wb.step();
    // WALK_F = steps per second (1.8 walk, 2.2 brisk, 2.8 jog), WALK_AMP scales the sway and the
    // accelerations, HEEL scales the heel strike
    static const double f2 = envd("WALK_F", 1.8), amp = envd("WALK_AMP", 1.0), hs = envd("HEEL", 1.0);
    double cyc = gg.step(t, f2), th2 = 2 * M_PI * cyc, th1 = th2 / 2;  // step and stride angles
    double ramp = std::min(1.0, std::max(0.0, (t - 30.0) / 2.0));
    double ph = fmod(cyc, 1.0), wd = 0.06 * f2;  // heel strike: 60 ms bump after each step
    double heel = ph < wd ? 0.5 - 0.5 * cos(2 * M_PI * ph / wd) : 0;
    if (getenv("J_NOHEEL")) heel = 0;
    w.leanS = 10 * ramp + amp * (3.5 * sin(th1) + 0.8 * sin(th2 + 0.7)) + wb.s;
    w.leanF = 8 * ramp + amp * (2.0 * sin(th2 + 1.0) + 0.6 * sin(2 * th2 + 0.3)) + hs * 0.8 * heel + wb.f;
    w.yaw = getenv("J_NOYAW") ? 0 : 5 * sin(th1);
    w.accLin[2] = amp * (0.22 * sin(th2) + 0.08 * sin(2 * th2)) + hs * 0.5 * heel;
    w.accLin[1] = amp * 0.15 * sin(th2 + 0.4);
    w.accLin[0] = amp * 0.08 * sin(th1 + 0.2);
    if (getenv("J_NOACC")) w.accLin[0] = w.accLin[1] = w.accLin[2] = 0;
    if (getenv("J_NOLEAN")) { w.leanS -= 10 * ramp; w.leanF -= 8 * ramp; }
  };
  auto tangga = [](double t, World &w) {
    static Gaitgen gg;
    static Wobble wb;
    wb.step();
    double st = std::min(1.0, std::max(0.0, t - (32.0 + WARM)));  // 0 = flat, 1 = stairs
    // STAIR_LIFT scales how much bigger the stride's motion is on stairs than on flat ground (a taller
    // step or a steeper flight); STAIR_ASYM scales the right leg's bigger lift over the left's that is
    // already there (a limp, or a spiral stairway). Both default to 1.0, reproducing the numbers below
    // exactly (pF/rF = flat, pF+pEx/rF+rEx = full stairs at st=1).
    double lift = envd("STAIR_LIFT", 1.0), asym = envd("STAIR_ASYM", 1.0);
    double cyc = gg.step(t, 2 * (0.9 - 0.2 * st)) / 2;             // strides
    double th = 2 * M_PI * cyc, ph = fmod(cyc, 1.0);
    auto bump = [&](double c, double wd) {  // raised-cosine bump at stride phase c, wd wide (phase units)
      double d = fmod(ph - c + 1.5, 1.0) - 0.5;
      return fabs(d) < wd / 2 ? 0.5 + 0.5 * cos(2 * M_PI * d / wd) : 0.0;
    };
    double wd = 0.06 * 2 * (0.9 - 0.2 * st);  // 60 ms
    double hR = bump(0.0, wd), hL = bump(0.5, wd), hSym = hR + hL, hDiff = hR - hL;
    double ramp = std::min(1.0, std::max(0.0, (t - 30.0) / 2.0));
    double pF = 2.0 * sin(2 * th + 1.0) + 0.6 * sin(4 * th + 0.3) + 0.8 * hSym;
    double pEx = 1.5 * sin(2 * th + 1.0) - 0.6 * sin(4 * th + 0.3) + 1.2 * sin(th + 0.4) + 0.7 * hSym + 0.5 * asym * hDiff;
    double rF = 3.5 * sin(th) + 0.8 * sin(2 * th + 0.7);
    double rEx = 1.5 * sin(th) + 0.7 * sin(2 * th + 0.7) + 1.5 * sin(3 * th + 0.2);
    w.leanS = 10 * ramp + rF + st * lift * rEx + wb.s;
    w.leanF = 8 * ramp + pF + st * lift * pEx + wb.f;
    w.yaw = 5 * sin(th);
    w.accLin[2] = (0.22 + 0.13 * st * lift) * sin(2 * th) + 0.5 * hSym + 0.1 * st * asym * hDiff;
    w.accLin[1] = 0.15 * sin(2 * th + 0.4);
    w.accLin[0] = 0.08 * sin(th + 0.2);
  };
  // A flat-ground limp: same stride rate and sway as ordinary walking (jalan's rF/pF non-heel terms), but
  // the heel strike lands harder on the right leg than the left every stride, instead of the same on both
  // (jalan's single, leg-blind heel bump). Favouring a sore leg, odd footwear, or an uneven belt mount.
  auto pincang = [](double t, World &w) {
    static Gaitgen gg;
    static Wobble wb;
    wb.step();
    static const double f2 = 1.8;  // strides/second, like ordinary walking
    double cyc = gg.step(t, f2) / 2, th = 2 * M_PI * cyc, ph = fmod(cyc, 1.0);  // strides: left and right differ
    auto bump = [&](double c, double wd) {
      double d = fmod(ph - c + 1.5, 1.0) - 0.5;
      return fabs(d) < wd / 2 ? 0.5 + 0.5 * cos(2 * M_PI * d / wd) : 0.0;
    };
    double wd = 0.06 * f2, hR = bump(0.0, wd), hL = bump(0.5, wd);
    double ramp = std::min(1.0, std::max(0.0, (t - 30.0) / 2.0));
    w.leanS = 10 * ramp + 3.5 * sin(th) + 0.8 * sin(2 * th + 0.7) + wb.s;
    w.leanF = 8 * ramp + 2.0 * sin(2 * th + 1.0) + 0.6 * sin(4 * th + 0.3) + 2.2 * hR + 0.4 * hL + wb.f;
    w.yaw = 5 * sin(th);
    w.accLin[2] = 0.22 * sin(2 * th) + 0.75 * hR + 0.25 * hL;
    w.accLin[1] = 0.15 * sin(th + 0.4);
    w.accLin[0] = 0.08 * sin(th / 2 + 0.2);
  };
  // Brisk/jog pace, promoted from a CI "summary only" sweep (WALK_F=2.8 WALK_AMP=2.5 HEEL=3, still in
  // firmware.yml) to a real, gated scenario: a faster stride, a bigger sway and a harder heel strike than
  // ordinary walking.
  auto cepat = [](double t, World &w) {
    static Gaitgen gg;
    static Wobble wb;
    wb.step();
    static const double f2 = 2.8, amp = 2.5, hs = 3.0;
    double cyc = gg.step(t, f2), th2 = 2 * M_PI * cyc, th1 = th2 / 2;
    double ramp = std::min(1.0, std::max(0.0, (t - 30.0) / 2.0));
    double ph = fmod(cyc, 1.0), wd = 0.06 * f2;
    double heel = ph < wd ? 0.5 - 0.5 * cos(2 * M_PI * ph / wd) : 0;
    w.leanS = 10 * ramp + amp * (3.5 * sin(th1) + 0.8 * sin(th2 + 0.7)) + wb.s;
    w.leanF = 8 * ramp + amp * (2.0 * sin(th2 + 1.0) + 0.6 * sin(2 * th2 + 0.3)) + hs * 0.8 * heel + wb.f;
    w.yaw = 5 * sin(th1);
    w.accLin[2] = amp * (0.22 * sin(th2) + 0.08 * sin(2 * th2)) + hs * 0.5 * heel;
    w.accLin[1] = amp * 0.15 * sin(th2 + 0.4);
    w.accLin[0] = amp * 0.08 * sin(th1 + 0.2);
  };
  struct WalkRes { Stat sp, sr, cp, cr, ep, er, vp, vr, vcp, vcr; std::vector<double> jp, jr, jcp, jcr; double stride = 0, sway = 0; };
  // jolt: how far the camera jumps within 30 ms, 99th percentile (what a viewer sees as a jerk)
  auto p99 = [](std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    return v[(size_t)(0.99 * (v.size() - 1))];
  };
  Stat *estP = nullptr, *estR = nullptr;  // how wrong the IMU estimate is (measured - true)
  // fast shake = camera turning rate after removing what is slower than ~5 Hz (what blurs a video frame
  // and makes rolling-shutter jello); sampled every 1 ms
  auto measure = [&](Stat &a, Stat &b, double secs, Stat *va = nullptr, Stat *vb = nullptr, std::vector<double> *ja = nullptr,
                     std::vector<double> *jb = nullptr) {
    uint64_t last = 0, lastV = 0;
    double sx = 0, sy = 0, px = 0, py = 0;
    double ringX[30], ringY[30];
    int ri = 0, rn = 0;
    bool first = true;
    run(secs, [&]() {
      if (va && t_us - lastV >= 1000) {
        lastV = t_us;
        double x, y;
        truth(x, y);
        // DUMP=file: camera angles, servo angles, servo pulses and body lean every 1 ms while measuring
        static FILE *dump = getenv("DUMP") ? fopen(getenv("DUMP"), "w") : nullptr;
        if (dump) fprintf(dump, "%.4f %.4f %.4f %.3f %.3f %.2f %.2f %.3f %.3f\n", t_us * 1e-6, x, y, W.pitchS.ang, W.rollS.ang, W.pitchS.pulse, W.rollS.pulse, W.leanF, W.leanS);
        if (first) { sx = x; sy = y; first = false; }
        const double k = 1.0 - exp(-0.001 / 0.032);  // 5 Hz high-pass
        sx += (x - sx) * k;
        sy += (y - sy) * k;
        double hx = x - sx, hy = y - sy;
        va->add((hx - px) / 0.001);
        vb->add((hy - py) / 0.001);
        if (ja && rn == 30) {
          ja->push_back(fabs(x - ringX[ri]));
          jb->push_back(fabs(y - ringY[ri]));
        }
        ringX[ri] = x;
        ringY[ri] = y;
        ri = (ri + 1) % 30;
        if (rn < 30) rn++;
        px = hx;
        py = hy;
      }
      if (t_us - last >= 5000) {
        last = t_us;
        double x, y;
        truth(x, y);
        a.add(x - cfg.pitchSet);
        b.add(y - cfg.rollSet);
        if (estP) { estP->add(pitchMeas - x); estR->add(rollMeas - y); }
      }
    });
  };
  auto walkTest = [&](const char *name, std::function<void(double, World &)> f) {
    WalkRes r;
    double tStart = t_us * 1e-6;
    W.body = [&](double t, World &w) { f(t - tStart + 30.0, w); };
    run(WARM);
    estP = &r.ep; estR = &r.er;
    measure(r.sp, r.sr, 12.0, &r.vp, &r.vr, &r.jp, &r.jr);
    r.stride = gait.w / (2 * M_PI);
    r.sway = gait.amp;
    estP = estR = nullptr;
    usb("MODE center");
    run(1.0);
    measure(r.cp, r.cr, 12.0, &r.vcp, &r.vcr, &r.jcp, &r.jcr);
    usb("MODE stab");
    run(2.0);
    W.body = nullptr;
    W.leanF = W.leanS = W.yaw = 0;
    W.accLin[0] = W.accLin[1] = W.accLin[2] = 0;
    run(3.0);
    takeUsb();
    printf("   %-7s stabil: atas-bawah rms %.2f maks %.2f | kanan-kiri rms %.2f maks %.2f (deg)\n", name, r.sp.rms(), r.sp.mx, r.sr.rms(), r.sr.mx);
    printf("   %-7s salah baca sensor saat stabil: atas-bawah rms %.2f (rata2 %+.2f) maks %.2f | kanan-kiri rms %.2f (rata2 %+.2f) maks %.2f\n", name, r.ep.rms(), r.ep.mean(), r.ep.mx, r.er.rms(), r.er.mean(), r.er.mx);
    printf("   %-7s getar cepat (>5 Hz, deg/s rms): stabil %.1f / %.1f, tanpa %.1f / %.1f\n", name, r.vp.rms(), r.vr.rms(), r.vcp.rms(), r.vcr.rms());
    printf("   %-7s sentakan (lompatan dalam 30 ms, persentil 99, deg): stabil %.2f / %.2f, tanpa %.2f / %.2f\n", name, p99(r.jp), p99(r.jr), p99(r.jcp), p99(r.jcr));
    printf("   %-7s tanpa : atas-bawah rms %.2f maks %.2f | kanan-kiri rms %.2f maks %.2f (deg)\n", name, r.cp.rms(), r.cp.mx, r.cr.rms(), r.cr.mx);
    printf("   %-7s pola langkah: %.2f langkah ganda/s, ayunan %.1f deg\n", name, r.stride, r.sway);
    return r;
  };
  printf("== walking\n");
  WalkRes wg = walkTest("goyang", goyang);
  WalkRes wj = walkTest("jalan", jalan);
  WalkRes wt = walkTest("tangga", tangga);
  WalkRes wp = walkTest("pincang", pincang);
  WalkRes wc = walkTest("cepat", cepat);
  Stat sp = wj.sp, sr = wj.sr;
  check(wg.sp.rms() < wg.cp.rms() * 0.5 && wg.sr.rms() < wg.cr.rms() * 0.5, "goyang: stabilisation removes more than half the motion");
  check(wj.sp.rms() < wj.cp.rms() * 0.5 && wj.sr.rms() < wj.cr.rms() * 0.5, "jalan: stabilisation removes more than half the motion");
  check(wj.sp.rms() < 0.8 && wj.sr.rms() < 0.8, "jalan: camera within 0.8 deg rms while walking");
  check(wj.vp.rms() < 20 && wj.vr.rms() < 20, "jalan: servos add little fast shake (< 20 deg/s rms above 5 Hz)");
  check(fabs(wj.ep.mean()) < 0.3 && fabs(wj.er.mean()) < 0.3, "jalan: walking does not bias the IMU estimate (< 0.3 deg)");
  check(p99(wj.jr) < 0.6 * p99(wj.jcr), "jalan: kanan-kiri jolts less than 60% of no gimbal");
  check(p99(wj.jp) < 0.85 * p99(wj.jcp), "jalan: atas-bawah jolt at each step below 85% of no gimbal");
  check(wt.sp.rms() < 0.8 && wt.sr.rms() < 0.8, "tangga: camera within 0.8 deg rms on stairs with uneven left/right steps");
  check(p99(wt.jp) < 0.6 * p99(wt.jcp) && p99(wt.jr) < 0.6 * p99(wt.jcr), "tangga: jolts less than 60% of no gimbal");
  check(wp.sp.rms() < wp.cp.rms() * 0.5 && wp.sr.rms() < wp.cr.rms() * 0.5, "pincang: stabilisation removes more than half the motion");
  check(wp.sp.rms() < 0.8 && wp.sr.rms() < 0.8, "pincang: camera within 0.8 deg rms despite the left/right limp");
  check(p99(wp.jp) < 0.6 * p99(wp.jcp) && p99(wp.jr) < 0.6 * p99(wp.jcr), "pincang: jolts less than 60% of no gimbal");
  check(wc.sp.rms() < wc.cp.rms() * 0.5 && wc.sr.rms() < wc.cr.rms() * 0.5, "cepat: stabilisation removes more than half the motion");
  check(wc.sp.rms() < 1.2 && wc.sr.rms() < 0.7, "cepat: camera within 1.2/0.7 deg rms (atas-bawah/kanan-kiri) at a brisk pace");
  check(p99(wc.jr) < 0.6 * p99(wc.jcr), "cepat: kanan-kiri jolts less than 60% of no gimbal");
  check(p99(wc.jp) < 0.85 * p99(wc.jcp), "cepat: atas-bawah jolt at each step below 85% of no gimbal");
  // the same walk without the learned step pattern: the pattern must make the step jolt clearly smaller
  if (cfg.gait > 0) {
    float g0 = cfg.gait;
    usb("SET gait 0");
    WalkRes wn = walkTest("tanpa pola", jalan);
    char cmd[32];
    snprintf(cmd, sizeof cmd, "SET gait %.2f", g0);
    usb(cmd);
    run(0.2);
    takeUsb();
    printf("   pola langkah: sentakan atas-bawah %.2f -> %.2f deg (%.0f%% lebih kecil)\n", p99(wn.jp), p99(wj.jp), 100 * (1 - p99(wj.jp) / p99(wn.jp)));
    check(p99(wj.jp) < 0.9 * p99(wn.jp) && wj.sp.rms() < wn.sp.rms() + 0.05, "jalan: learned step pattern makes the step jolt at least 10% smaller, no worse on average");
  }
  W.body = nullptr;
  W.leanF = W.leanS = W.yaw = 0;
  W.accLin[0] = W.accLin[1] = W.accLin[2] = 0;
  // 8 s, not 4: cepat is brisker than the older three walks, and the PID integrator needs a little longer
  // to settle to its resting value before we check that the dead band compensation has stopped hunting.
  run(8.0);
  takeUsb();

  double jitP = 0, jitR = 0;
  {
    double s3 = 0, s33 = 0, s4 = 0, s44 = 0; long n = 0;
    run(3.0, [&]() {
      static uint64_t last = 0;
      if (t_us - last >= 5000) { last = t_us; s3 += pinPulse[3]; s33 += pinPulse[3] * pinPulse[3]; s4 += pinPulse[4]; s44 += pinPulse[4] * pinPulse[4]; n++; }
    });
    jitP = sqrt(std::max(0.0, s33 / n - (s3 / n) * (s3 / n)));
    jitR = sqrt(std::max(0.0, s44 / n - (s4 / n) * (s4 / n)));
    printf("   servo jitter when still: %.2f / %.2f us rms\n", jitP, jitR);
    check(jitP < 1.5 && jitR < 1.5, "servos quiet when still (< 1.5 us rms)");
  }
  double riseMs = -1, overPct = -1;
  printf("== step test (STEP p 5)\n");
  usb("STEP p 5");
  run(8.0);
  o = takeUsb();
  {
    std::vector<std::array<int, 8>> rows;
    for (auto &l : lines(o)) {
      if (l.rfind("C,", 0) == 0) {
        std::array<int, 8> r;
        sscanf(l.c_str(), "C,%d,%d,%d,%d,%d,%d,%d,%d", &r[0], &r[1], &r[2], &r[3], &r[4], &r[5], &r[6], &r[7]);
        rows.push_back(r);
      }
    }
    check(rows.size() == 300 && o.find("CAPEND") != std::string::npos, "300 samples dumped");
    // time until the camera is within 1 deg of the new target (a servo's dead band can leave the last
    // ~0.7 deg to the slower I term), overshoot, and how close it is just before the step ends
    double base = 35.0, tIn = -1, peak = 0, endErr = 9;
    for (auto &r : rows) {
      double t = r[1] / 1000.0, p = r[3] / 100.0;
      if (t < 0.5 || t > 2.0) continue;
      double f = (p - base) / 5.0;
      if (tIn < 0 && fabs(p - base - 5.0) < 1.0) tIn = t - 0.5;
      peak = std::max(peak, f);
      if (t > 1.85) endErr = fabs(p - base - 5.0);
    }
    printf("   within 1 deg after %.0f ms, overshoot %.0f%%, off by %.2f deg after 1.4 s\n", tIn * 1000, (peak - 1) * 100, endErr);
    riseMs = tIn * 1000; overPct = (peak - 1) * 100;
    printf("SUMMARY | goyang rms p %.2f r %.2f | jalan rms p %.2f r %.2f max p %.2f r %.2f | reach %.0f ms over %.0f%% | jitter %.2f %.2f us | getar %.1f %.1f (tanpa %.1f %.1f) | sentak %.2f %.2f (tanpa %.2f %.2f) | tangga rms %.2f %.2f sentak %.2f %.2f\n",
      wg.sp.rms(), wg.sr.rms(), sp.rms(), sr.rms(), sp.mx, sr.mx, riseMs, overPct, jitP, jitR, wj.vp.rms(), wj.vr.rms(), wj.vcp.rms(), wj.vcr.rms(),
      p99(wj.jp), p99(wj.jr), p99(wj.jcp), p99(wj.jcr), wt.sp.rms(), wt.sr.rms(), p99(wt.jp), p99(wt.jr));
    if (getenv("QUICK")) return 0;
    check(tIn > 0 && tIn < 0.3 && peak < 1.25 && endErr < 0.5, "step: within 1 deg in 0.3 s, overshoot < 25%, settles within 0.5 deg");
  }

  printf("== handheld: target 0 (straight ahead), body leans far and comes back\n");
  {
    usb("SET pitch 0");
    usb("SET roll 0");
    run(3.0);
    // sideways 50 deg both ways, nose up 25, nose down 10, and a mix (leanF < 0 = body leans forward).
    // The pitch servo's centre already looks 37 deg down in this frame, so straight ahead needs -37 deg
    // and a forward lean of more than ~13 deg runs into the +-50 deg servo limit.
    struct Pose { double s, f; const char *name; } poses[] = {
        {50, 0, "miring kanan 50"}, {-50, 0, "miring kiri 50"}, {0, 25, "mendongak 25"}, {0, -10, "menunduk 10"}, {30, 15, "miring 30 + mendongak 15"}};
    for (auto &ps : poses) {
      double t0 = t_us * 1e-6, s0 = W.leanS, f0 = W.leanF;
      W.body = [=](double t, World &w) {
        double r = std::min(1.0, std::max(0.0, (t - t0) / 1.0));  // lean over 1 s, then hold
        w.leanS = s0 + (ps.s - s0) * r;
        w.leanF = f0 + (ps.f - f0) * r;
      };
      run(3.0);
      truth(tp, tr);
      printf("   %-26s camera pitch %+6.2f roll %+6.2f\n", ps.name, tp, tr);
      char msg[96];
      snprintf(msg, sizeof msg, "handheld %s: camera back to straight/level within 1 deg", ps.name);
      check(fabs(tp) < 1.0 && fabs(tr) < 1.0, msg);
    }
    W.body = nullptr;
    W.leanF = W.leanS = W.yaw = 0;
    usb("SET pitch 35");
    run(3.0);
    takeUsb();
  }

  printf("== WiFi: first start\n");
  check(WiFi.ap && WiFi.apName == "Gimbal-D435-A1B2" && WiFi.apPassword == "gimbald435", "no hotspot saved yet: own WiFi Gimbal-D435-A1B2 (sandi gimbald435)");
  check(MDNS.host == "gimbal-d435" && MDNS.services == 1, "address gimbal-d435.local");
  check(WiFi.txPower == WIFI_POWER_8_5dBm && !WiFi.sleep, "low WiFi power (less heat), no WiFi sleep (quick replies)");

  printf("== the app is served by the gimbal itself\n");
  {
    std::string r = httpGet("/");
    size_t h = r.find("\r\n\r\n");
    std::string body = h == std::string::npos ? "" : r.substr(h + 4);
    check(r.rfind("HTTP/1.1 200 OK", 0) == 0 && r.find("Content-Encoding: gzip") != std::string::npos && body.size() == WEB_FILES[0].len &&
              (uint8_t)body[0] == 0x1f && (uint8_t)body[1] == 0x8b,
          "GET / -> the app (gzip, complete)");
    r = httpGet("/", std::string("If-None-Match: ") + WEB_FILES[0].etag + "\r\n");
    check(r.rfind("HTTP/1.1 304", 0) == 0, "unchanged file -> 304, the browser uses its copy");
    r = httpGet("/vendor/three.module.min.js");
    h = r.find("\r\n\r\n");
    check(r.rfind("HTTP/1.1 200 OK", 0) == 0 && r.size() - h - 4 == WEB_FILES[1].len, "3D library (163 KB) sent in pieces");
    check(httpGet("/rahasia").rfind("HTTP/1.1 404", 0) == 0, "unknown file -> 404");
  }

  printf("== WebSocket: PIN, two devices, slow device, lockout\n");
  auto a = wsOpen();
  check(a->out.find("Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=") != std::string::npos, "WebSocket handshake (RFC 6455 example key)");
  wsSend(a, "HELLO");
  run(0.2);
  std::string b = wsTake(a);
  show(b);
  std::string nonce = nonceOf(b, "\"nonce\":\"");
  check(nonce.size() == 32 && b.find("\"auth\":1") != std::string::npos, "HELLO gives a nonce, PIN required");
  wsSend(a, "GET");
  run(0.2);
  b = wsTake(a);
  check(b.find("ERR AUTH") != std::string::npos && b.find("CFG") == std::string::npos, "settings refused before the PIN");
  char h[65];
  authHash(nonce.c_str(), "000000", h);
  wsSend(a, std::string("AUTH ") + h);
  run(0.2);
  b = wsTake(a);
  check(b.find("AUTH FAIL 4") != std::string::npos, "wrong PIN refused");
  nonce = nonceOf(b, "AUTH FAIL 4 ");
  authHash(nonce.c_str(), "123456", h);
  wsSend(a, std::string("AUTH ") + h);
  run(1.5);
  b = wsTake(a);
  check(b.find("AUTH OK") != std::string::npos && ws[0].authed && b.find("CFG {") != std::string::npos, "correct PIN accepted, settings sent");
  printf("   telemetry lines in 1.5 s over WiFi: %d\n", countT(b));
  check(countT(b) >= 27, "telemetry 20 times per second");
  auto d2 = wsOpen("x3JJHMbDL1EzLkh9GBhXDw==");
  wsAuth(d2, "123456");
  b = wsTake(d2);
  check(b.find("AUTH OK") != std::string::npos && wsCount(true) == 2, "a second phone/iPad can connect at the same time");
  wsSend(a, "SET pitch 30");
  run(4.0);
  std::string ba = wsTake(a), bb = wsTake(d2);
  truth(tp, tr);
  check(ba.find("\"pitch\":30.00") != std::string::npos && bb.find("\"pitch\":30.00") != std::string::npos, "a change from one device shows on the other");
  check(fabs(cfg.pitchSet - 30) < 0.01 && fabs(tp - 30) < 0.3, "SET pitch 30 from the app moves the camera to 30 deg");
  d2->room = 0;  // the iPad stops reading (screen locked, weak signal)
  uint32_t loops0 = loopCount;
  run(3.0);
  b = wsTake(a);
  check(countT(b) >= 55 && ws[1].used, "a stalled device does not slow the gimbal or the other device");
  d2->room = 1 << 20;
  wsSend(d2, "PING");
  run(1.0);
  b = wsTake(d2);
  check(b.find("PONG") != std::string::npos && countT(b) > 0, "it catches up when it reads again");
  (void)loops0;
  d2->peerOpen = false;
  run(0.2);
  check(!ws[1].used && wsCount(true) == 1, "closed device frees its place");
  {
    auto x1 = wsOpen("AAAAAAAAAAAAAAAAAAAAAA=="), x2 = wsOpen("BBBBBBBBBBBBBBBBBBBBBB=="), x3 = wsOpen("CCCCCCCCCCCCCCCCCCCCCC==");
    check(x3->out.rfind("HTTP/1.1 503", 0) == 0, "4th device at once gets a clear 'full' answer");
    x1->peerOpen = x2->peerOpen = false;
    run(0.2);
  }
  {
    auto c = wsOpen("DDDDDDDDDDDDDDDDDDDDDD==");
    wsSend(c, "HELLO");
    run(0.2);
    for (int i = 0; i < 5; i++) {
      wsSend(c, "AUTH 0000000000000000000000000000000000000000000000000000000000000000");
      run(0.2);
    }
    run(0.6);
    b = wsTake(c);
    check(b.find("AUTH LOCKED 30") != std::string::npos && !c->open, "5 wrong PINs -> locked 30 s and disconnected");
    lockUntil = 0;
  }
  {
    auto q = wsOpen("EEEEEEEEEEEEEEEEEEEEEE==");
    for (int i = 0; i < 5; i++) {  // the first device keeps pinging like the app does
      wsSend(a, "PING");
      run(5.0);
    }
    check(!q->open && ws[0].used, "a silent connection is dropped after 20 s, the pinging app stays");
  }

  printf("== WiFi: join the phone hotspot from the app\n");
  WiFi.apStations = 1;  // the phone doing the setup is on the gimbal's own WiFi
  wsSend(a, "WIFI SCAN");
  run(2.5);
  b = wsTake(a);
  show(b, "WIFI LIST");
  check(b.find("WIFI LIST [{\"s\":\"HP Khair\"") != std::string::npos && b.find("Wifi \\\"Rumah\\\"") != std::string::npos &&
            b.find("HP Khair", b.find("HP Khair") + 1) == std::string::npos,
        "scan lists hotspots once each, names escaped");
  wsSend(a, "WIFI JOIN " + hexs("HP Khair") + " " + hexs("rahasia123"));
  run(3.0);
  b = wsTake(a);
  show(b, "Tersambung");
  check(staUp && !strcmp(cfg.wifiSsid, "HP Khair") && b.find("http://gimbal-d435.local atau http://172.20.10.3") != std::string::npos,
        "joins the hotspot and tells the new address");
  check(WiFi.ap, "own WiFi stays on while the phone is still on it (to read the address)");
  for (int i = 0; i < 25 && WiFi.ap; i++) {
    wsSend(a, "PING");
    run(5.0);
  }
  check(!WiFi.ap && WiFi.modeNow == WIFI_STA, "then the own WiFi turns off (one radio, less heat)");
  WiFi.apStations = 0;
  WiFi.hotspotOn = false;  // hotspot switched off
  for (int i = 0; i < 5; i++) {
    wsSend(a, "PING");
    run(5.0);
  }
  check(!staUp && WiFi.ap, "hotspot gone for 20 s -> own WiFi back on for setup");
  WiFi.hotspotOn = true;
  for (int i = 0; i < 6 && !(staUp && !WiFi.ap); i++) {
    wsSend(a, "PING");
    run(5.0);
  }
  check(staUp && !WiFi.ap, "hotspot back -> joins again, own WiFi off");
  wsSend(a, "WIFI JOIN 41 3132");
  run(0.2);
  check(wsTake(a).find("ERR sandi hotspot minimal 8") != std::string::npos, "too-short hotspot password refused");

  printf("== IMU wire breaks for 2 s\n");
  takeUsb();
  double pBefore = pinPulse[4];
  W.imuConnected = false;
  run(2.0);
  double pDuring = pinPulse[4];
  W.imuConnected = true;
  run(2.5);
  o = takeUsb();
  show(o, "EV");
  check(o.find("IMU ERROR") != std::string::npos && o.find("IMU terbaca lagi") != std::string::npos, "IMU loss reported and recovered");
  check(fabs(pDuring - pBefore) < 15, "servo holds its position while the IMU is lost");

  printf("== persistence: reboot keeps calibration\n");
  Config before = cfg;
  memset(&cfg, 0, sizeof(cfg));
  check(cfgLoad() && cfg.usDegP == before.usDegP && cfg.swapServo == before.swapServo && fabs(cfg.pitchSet - 30) < 0.01, "config reloaded from flash");

  printf("== upgrade: settings saved by firmware 4.0 are kept\n");
  {
    ConfigV40 o;
    memset(&o, 0, sizeof(o));
    memcpy(&o, &before, offsetof(ConfigV40, crc));  // 4.0 layout = same fields up to the old crc
    o.size = sizeof(ConfigV40);
    o.crc = crc32((const uint8_t *)&o, offsetof(ConfigV40, crc));
    prefs.putBytes("cfg", &o, sizeof(o));
    memset(&cfg, 0, sizeof(cfg));
    cfgMigrated = nullptr;
    bool ok = cfgLoad();
    check(ok && cfgMigrated && cfg.size == sizeof(Config) && cfg.usDegP == before.usDegP && cfg.swapServo == before.swapServo &&
              fabs(cfg.pitchSet - 30) < 0.01 && fabs(cfg.ffBody - 1.0f) < 1e-6 && fabs(cfg.srvTau - 25.0f) < 1e-6 && !strcmp(cfg.pin, before.pin),
          "4.0 settings kept, new settings get defaults");
  }

  printf("== upgrade: settings saved by firmware 4.1 are kept\n");
  {
    ConfigV41 o;
    memset(&o, 0, sizeof(o));
    memcpy(&o, &before, offsetof(ConfigV41, crc));  // 4.1 layout = same fields up to its crc
    o.size = sizeof(ConfigV41);
    o.srvTau = 31.0f;
    o.crc = crc32((const uint8_t *)&o, offsetof(ConfigV41, crc));
    prefs.putBytes("cfg", &o, sizeof(o));
    memset(&cfg, 0, sizeof(cfg));
    cfgMigrated = nullptr;
    bool ok = cfgLoad();
    check(ok && cfgMigrated && !strcmp(cfgMigrated, "4.1") && cfg.size == sizeof(Config) && cfg.usDegP == before.usDegP &&
              fabs(cfg.srvTau - 31.0f) < 1e-6 && !strcmp(cfg.wifiSsid, before.wifiSsid) && fabs(cfg.gait - GAIT_DEFAULT) < 1e-6,
          "4.1 settings kept (WiFi too), walking pattern gets its default");
  }

  printf("\n%s (%d failed)\n", fails ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED", fails);
  return fails ? 1 : 0;
}
