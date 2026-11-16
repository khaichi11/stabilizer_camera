// Host simulation of the gimbal: runs the real firmware (.ino) against a physics model of
// the two servos, the camera, an MPU6050 mounted upside-down and rotated, body motion while walking.
#include "Arduino.h"
#include "Wire.h"
#include "Preferences.h"
#include "BLEDevice.h"
#include "esp_system.h"
#include "esp_random.h"
#include <random>
#include <functional>
#include <map>

// ------------------------------------------------------------------ sim clock + physics state
static uint64_t t_us = 0;
static std::mt19937 rng(12345);
static std::normal_distribution<double> nd(0.0, 1.0);
MockSerial Serial;
MockESP ESP;
TwoWire Wire;
BLEServer *BLEDevice::srv = nullptr;
int BLEDevice::advStarts = 0;
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

struct Servo {
  double ang = 0;      // deg, output shaft (after backlash)
  double motor = 0;    // deg, motor side
  double pulse = 0;    // us, 0 = no pulses
  double usPerDeg = 11.0;
  void step(double dt) {
    if (pulse <= 0) return;
    double tgt = (pulse - 1500.0) / usPerDeg;
    double err = tgt - motor;
    if (fabs(err) > 8.0 / usPerDeg) {  // servo dead band (8 us)
      double v = err / 0.025;           // 25 ms lag
      v = std::max(-500.0, std::min(500.0, v));
      motor += v * dt;
    }
    double bl = getenv("BACKLASH") ? atof(getenv("BACKLASH")) : 0.0;
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
  W.pitchS.step(dt);
  W.rollS.step(dt);
  computeOrientation();
  if (W.havePrev) {  // omega from R_prev^T R_now (IMU frame)
    M3 rt, S;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) rt[i][j] = W.Rimu_prev[j][i];
    mmul(rt, W.Rimu, S);
    double om[3] = {(S[2][1] - S[1][2]) / (2 * dt) / D2R, (S[0][2] - S[2][0]) / (2 * dt) / D2R, (S[1][0] - S[0][1]) / (2 * dt) / D2R};
    double k = dt / (0.0045 + dt);
    for (int i = 0; i < 3; i++) W.omega[i] += (om[i] - W.omega[i]) * k;
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
bool ledcAttach(uint8_t, uint32_t, uint8_t) { return true; }
static std::map<int, double> pinPulse;
bool ledcWrite(uint8_t pin, uint32_t duty) {
  double us = duty * 20000.0 / 16383.0;
  pinPulse[pin] = us;
  Servo &s3 = W.swapped ? W.rollS : W.pitchS;
  Servo &s4 = W.swapped ? W.pitchS : W.rollS;
  if (pin == 3) s3.pulse = us;
  if (pin == 4) s4.pulse = us;
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
static uint8_t regs[128];
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
      put16(rBuf, (v + 0.004 * nd(rng)) * 8192.0);
    }
    put16(rBuf, 0);  // temperature
    for (int k = 0; k < 3; k++) put16(rBuf, (W.omega[k] + W.gyroBias[k] + 0.05 * nd(rng)) * 65.5);
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
struct Stat { double sum2 = 0, mx = 0; long n = 0; void add(double e) { sum2 += e * e; mx = std::max(mx, fabs(e)); n++; } double rms() const { return n ? sqrt(sum2 / n) : 0; } };

static BLECharacteristic *chr(const char *uuid) {
  for (auto c : BLEDevice::srv->svc->chars)
    if (c->uuid == uuid) return c;
  return nullptr;
}
static std::string bleRx;
static void bleWrite(const std::string &s) {
  auto rxc = chr(NUS_RX);
  for (size_t i = 0; i < s.size(); i += 20) {  // 20-byte writes like a small MTU
    rxc->value = s.substr(i, 20);
    rxc->cb->onWrite(rxc);
  }
}
static std::string bleTake() {
  auto txc = chr(NUS_TX);
  for (auto &p : txc->notified) bleRx += p;
  txc->notified.clear();
  std::string s = bleRx;
  bleRx.clear();
  return s;
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
  run(16.0);
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
  check(fabs(cfg.usDegP - W.pitchS.usPerDeg) < 0.8 && fabs(cfg.usDegR - W.rollS.usPerDeg) < 0.8, "servo scale within 0.8 us/deg");
  printf("   bias %.2f %.2f %.2f (true %.2f %.2f %.2f)\n", cfg.bias[0], cfg.bias[1], cfg.bias[2], W.gyroBias[0], W.gyroBias[1], W.gyroBias[2]);

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
    const char *keys[] = {"kp", "gain", "ff", "smooth", "db", "acctau"};
    for (auto k : keys) {
      std::string env = k; for (auto &c : env) c = toupper(c);
      if (getenv(env.c_str())) { std::string c = std::string("SET ") + k + " " + getenv(env.c_str()); usb(c.c_str()); }
    }
  }
  run(5.0);
  truth(tp, tr);
  printf("   true camera after levelling: pitch %.2f roll %.2f\n", tp, tr);
  check(fabs(tp - 35) < 0.25 && fabs(tr) < 0.25, "camera truly at 35.0 / 0.0 within 0.25 deg");

  printf("== walking: body leans 10 deg sideways + 8 deg forward, sway 3 deg @1.8 Hz, bumps 0.3 g\n");
  auto walk = [](double t, World &w) {
    double t0 = 30.0;
    double ramp = std::min(1.0, std::max(0.0, (t - t0) / 2.0));
    w.leanS = 10 * ramp + 3 * sin(2 * M_PI * 1.8 * t);
    w.leanF = 8 * ramp + 2.5 * sin(2 * M_PI * 1.8 * t + 1.0);
    w.yaw = 6 * sin(2 * M_PI * 0.9 * t);
    w.accLin[2] = 0.3 * sin(2 * M_PI * 3.6 * t);
    w.accLin[1] = 0.12 * sin(2 * M_PI * 1.8 * t + 0.4);
  };
  double tStart = t_us * 1e-6;
  W.body = [&](double t, World &w) { walk(t - tStart + 30.0, w); };
  Stat sp, sr;
  run(4.0);
  run(10.0, [&]() {
    static uint64_t last = 0;
    if (t_us - last >= 5000) {
      last = t_us;
      double a, b;
      truth(a, b);
      sp.add(a - 35);
      sr.add(b);
    }
  });
  printf("   stabilised:   pitch error rms %.2f max %.2f | roll error rms %.2f max %.2f (deg)\n", sp.rms(), sp.mx, sr.rms(), sr.mx);
  {
    usb("MODE center");
    run(1.0);
    Stat cp, cr;
    run(10.0, [&]() {
      static uint64_t last = 0;
      if (t_us - last >= 5000) {
        last = t_us;
        double a, b;
        truth(a, b);
        cp.add(a - 35);
        cr.add(b);
      }
    });
    printf("   no gimbal:    pitch error rms %.2f max %.2f | roll error rms %.2f max %.2f (deg)\n", cp.rms(), cp.mx, cr.rms(), cr.mx);
    check(sp.rms() < cp.rms() * 0.5 && sr.rms() < cr.rms() * 0.5, "stabilisation removes more than half the motion");
    usb("MODE stab");
    run(2.0);
  }
  W.body = nullptr;
  W.leanF = W.leanS = W.yaw = 0;
  W.accLin[0] = W.accLin[1] = W.accLin[2] = 0;
  run(4.0);
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
    double base = 35.0, t10 = -1, t90 = -1, peak = 0;
    for (auto &r : rows) {
      double t = r[1] / 1000.0, p = r[3] / 100.0;
      if (t < 0.5 || t > 2.0) continue;
      double f = (p - base) / 5.0;
      if (t10 < 0 && f >= 0.1) t10 = t;
      if (t90 < 0 && f >= 0.9) t90 = t;
      peak = std::max(peak, f);
    }
    printf("   rise 10-90%% %.0f ms, overshoot %.0f%%\n", (t90 - t10) * 1000, (peak - 1) * 100);
    riseMs = (t90 - t10) * 1000; overPct = (peak - 1) * 100;
    printf("SUMMARY kp=%s ki=%s ff=%s smooth=%s | walk rms p %.2f r %.2f max p %.2f r %.2f | rise %.0f ms over %.0f%% | jitter %.2f %.2f us\n",
      getenv("KP") ? getenv("KP") : "-", getenv("GAIN") ? getenv("GAIN") : "-", getenv("FF") ? getenv("FF") : "-", getenv("SMOOTH") ? getenv("SMOOTH") : "-",
      sp.rms(), sr.rms(), sp.mx, sr.mx, riseMs, overPct, jitP, jitR);
    if (getenv("QUICK")) return 0;
    check(t90 > 0 && (t90 - t10) < 0.4 && peak < 1.25, "step response fast, overshoot < 25%");
  }

  printf("== Bluetooth: connect, PIN check, lockout\n");
  BLEDevice::srv->mtu = 23;
  BLEDevice::srv->cb->onConnect(BLEDevice::srv);
  run(0.05);
  bleWrite("HELLO\n");
  run(0.3);
  std::string b = bleTake();
  show(b);
  std::string nonce;
  {
    size_t p = b.find("\"nonce\":\"");
    if (p != std::string::npos) nonce = b.substr(p + 9, 32);
  }
  check(nonce.size() == 32 && b.find("\"auth\":1") != std::string::npos, "HELLO gives a nonce, auth required");
  bleWrite("GET\n");
  run(0.2);
  b = bleTake();
  check(b.find("ERR AUTH") != std::string::npos && b.find("CFG") == std::string::npos, "settings refused before PIN");
  char h[65];
  authHash(nonce.c_str(), "000000", h);
  bleWrite(std::string("AUTH ") + h + "\n");
  run(0.2);
  b = bleTake();
  show(b);
  check(b.find("AUTH FAIL 4") != std::string::npos, "wrong PIN refused");
  {
    size_t p = b.find("AUTH FAIL 4 ");
    nonce = b.substr(p + 12, 32);  // new nonce after a failure
  }
  authHash(nonce.c_str(), "123456", h);
  bleWrite(std::string("AUTH ") + h + "\n");
  run(0.5);
  b = bleTake();
  show(b, "AUTH");
  check(b.find("AUTH OK") != std::string::npos && bleAuthed, "correct PIN accepted");
  bleWrite("GET\n");
  run(1.5);
  b = bleTake();
  check(b.find("CFG {") != std::string::npos, "CFG over Bluetooth (chunked 20 bytes)");
  int tcount = 0;
  for (auto &l : lines(b))
    if (l.rfind("T,", 0) == 0) tcount++;
  printf("   telemetry lines in 1.5 s over BLE: %d\n", tcount);
  check(tcount >= 10, "telemetry flows over Bluetooth");
  bleWrite("SET pitch 30\n");
  run(4.0);
  bleTake();
  truth(tp, tr);
  check(fabs(cfg.pitchSet - 30) < 0.01 && fabs(tp - 30) < 0.3, "SET pitch 30 from the app moves the camera to 30 deg");
  BLEDevice::srv->cb->onDisconnect(BLEDevice::srv);
  int adv0 = BLEDevice::advStarts;
  run(0.1);
  check(BLEDevice::advStarts == adv0 + 1 && !bleAuthed, "advertising restarts after disconnect");
  // lockout
  BLEDevice::srv->cb->onConnect(BLEDevice::srv);
  run(0.05);
  bleWrite("HELLO\n");
  run(0.2);
  bleTake();
  for (int i = 0; i < 5; i++) {
    bleWrite("AUTH 0000000000000000000000000000000000000000000000000000000000000000\n");
    run(0.2);
  }
  run(0.6);
  b = bleTake();
  check(b.find("AUTH LOCKED 30") != std::string::npos && BLEDevice::srv->disconnects == 1, "5 wrong PINs -> locked + disconnected");
  BLEDevice::srv->cb->onDisconnect(BLEDevice::srv);
  run(0.1);

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

  printf("\n%s (%d failed)\n", fails ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED", fails);
  return fails ? 1 : 0;
}
