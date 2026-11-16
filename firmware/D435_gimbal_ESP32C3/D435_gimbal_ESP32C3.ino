/*
  ============================================================================================
  GIMBAL D435 (sabuk) - firmware v4.0
  ESP32-C3 Super Mini + MPU6050 (GY-521) + 2x MG90S 180 derajat
  ============================================================================================
  Fitur
   - Menstabilkan nunduk (pitch) dan kiri-kanan (roll). IMU menempel di bawah dudukan kamera,
     jadi yang diukur adalah kamera itu sendiri (closed loop).
   - Dikendalikan dari aplikasi web lewat Bluetooth (BLE, Nordic UART Service) atau kabel USB.
     Bluetooth dilindungi sandi (PIN, challenge-response SHA-256: sandi tidak pernah dikirim).
   - Semua pengaturan tersimpan di flash dan tetap ada walau listrik dicabut.
   - Kalibrasi otomatis: arah sumbu IMU, kabel servo tertukar, arah dan skala tiap servo.
   - Kalibrasi datar (pakai waterpass / aplikasi waterpass di HP) dan kalibrasi gyro.
   - Uji respons (step test), log kejadian, penyebab restart (termasuk tegangan turun).
   - Servo menyala bergantian dan bergerak pelan di awal (lonjakan arus kecil).
   - LED biru (GPIO8): kedip pendek tiap 2 dtk = menunggu Bluetooth, kedip 1 Hz = tersambung
     tapi belum masuk sandi, NYALA TERUS = tersambung dan sandi benar, kedip cepat = IMU bermasalah.

  Kabel (ESP32-C3 Super Mini)
    MPU6050  VCC -> 3V3   GND -> G   SDA -> GPIO6   SCL -> GPIO7
    Servo nunduk (di G_Base)          oranye -> GPIO3
    Servo kiri-kanan (di belakang kamera) oranye -> GPIO4   (kalau tertukar, kalibrasi otomatis membetulkannya)
    Merah kedua servo -> 5V,  coklat kedua servo -> G,  kapasitor 100-470 uF di 5V/G
    Listrik: power bank 5 V >= 2 A -> kabel USB-A ke USB-C -> port USB-C ESP32.  Tanpa resistor.

  Arduino IDE
    - Boards Manager: "esp32" by Espressif Systems, versi 3.x
    - Tools > Board: "ESP32C3 Dev Module"
    - Tools > USB CDC On Boot: "Enabled"
    - Partition Scheme bawaan sudah cukup (firmware +-711 KB)
    - Tidak perlu library tambahan.

  Sandi Bluetooth bawaan: 123456  (ganti di aplikasi, menu Keamanan)
  Lupa sandi: colok USB, buka Serial Monitor 115200, kirim:  PIN 123456
  Perintah teks (USB atau Bluetooth): kirim HELP.
*/
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#if defined(CONFIG_BLUEDROID_ENABLED)
#include <BLE2902.h>
#endif
#include <stdarg.h>
#include "esp_system.h"
#include "esp_random.h"

#define FW_VERSION "4.0"

// ------------------------------------------------------------------------------------ pins
static const int PIN_SDA = 6, PIN_SCL = 7;
static const int PIN_SERVO_A = 3;   // normally the pitch servo (nunduk, in G_Base)
static const int PIN_SERVO_B = 4;   // normally the roll servo (kiri-kanan, behind the camera)
static const int PIN_LED = 8;       // onboard blue LED, active LOW

// ------------------------------------------------------------------------------------ BLE: Nordic UART Service
#define NUS_SERVICE "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // app -> gimbal (write)
#define NUS_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // gimbal -> app (notify)

// ------------------------------------------------------------------------------------ constants
static const uint32_t LOOP_US = 5000;  // 200 Hz control loop
static const float CAL_DUS = 150.0f;   // auto-calibration servo move (us), about 14 degrees
static const char *DEFAULT_PIN = "123456";
static const char *DEFAULT_NAME = "Gimbal D435";

enum Mode : uint8_t { MODE_STAB = 0, MODE_CENTER = 1, MODE_MANUAL = 2, MODE_OFF = 3 };
static const char *MODE_NAMES[] = { "stab", "center", "manual", "off" };
enum : uint8_t { TO_USB = 1, TO_BLE = 2 };

// ------------------------------------------------------------------------------------ configuration (flash)
struct Config {
  uint32_t magic;
  uint16_t size;
  uint16_t calOK;  // bit0 = auto calibration done, bit1 = level done, bit2 = gyro done
  float pitchSet, rollSet;
  float kp, gain, rateFF, deadband, smooth, maxRate, accTau;  // PID: kp = P, gain = I, rateFF = D (gyro)
  uint8_t dlpf, autoBias, bootGyro, nod;
  int16_t trimP, trimR;
  int8_t dirP, dirR, swapServo, startMode;
  float usDegP, usDegR, limP, limR;
  int8_t axF, axL, axU, sgF, sgL, sgU, telHz, ledOn;
  float lvlP, lvlR;
  float bias[3];
  char name[24];
  char pin[16];
  uint32_t crc;
};
static const uint32_t CFG_MAGIC = 0x34424D47;  // "GMB4"
Config cfg;
Preferences prefs;

// ------------------------------------------------------------------------------------ types
// (all types before the first function, so the Arduino IDE's automatic prototypes compile)
struct Sha256 {
  uint32_t h[8];
  uint8_t buf[64];
  uint64_t len;
  size_t n;
};
struct Ev {
  uint32_t t;
  char s[72];
};
enum CalKind : uint8_t { CAL_IDLE = 0, CAL_GYRO, CAL_LEVEL, CAL_AUTO };
static const char *CAL_NAMES[] = { "idle", "gyro", "level", "auto" };
struct Cal {
  CalKind kind = CAL_IDLE;
  uint8_t phase = 0, servo = 0, src = TO_USB;
  uint32_t t0 = 0;
  int n = 0;
  float s[3], ss[3], a[3];
  float thSum = 0, phSum = 0, refP = 0, refR = 0, gMax = 0;
  bool useP = true, useR = true;  // level: which axes to set
  float usA = 0, usB = 0;  // auto: pulse offsets (us) on GPIO3 / GPIO4
  float up[3];
  float rot[2][3], aBefore[2][3], aAfter[2][3];
} cal;
struct Sample {
  uint16_t t;
  int16_t sp, p, sr, r, op, orr;
};
static const int CAP_N = 300;  // 3 s at 100 Hz

// ------------------------------------------------------------------------------------ runtime state
// IMU
uint8_t imuAddr = 0x68, imuWho = 0;
bool imuOK = false;
uint32_t i2cErr = 0;
int imuFailRun = 0;
float accRaw[3] = { 0, 0, 1 };     // g, raw IMU axes
float gyrUncal[3] = { 0, 0, 0 };   // deg/s, raw IMU axes, no bias removed
float gyrRaw[3] = { 0, 0, 0 };     // deg/s, raw IMU axes, bias removed
// attitude
float thRaw = 0, phRaw = 0;        // estimator (deg), before level offsets
float pitchMeas = 0, rollMeas = 0; // after level offsets (deg)
float pitchRate = 0, rollRate = 0; // Euler rates (deg/s)
float stillTime = 0;
bool isStill = false;
uint32_t accCount = 0, accRejCount = 0;
float accRejPct = 0;
// control
Mode mode = MODE_STAB;
float cmdP = 0, cmdR = 0;    // controller output (deg of camera rotation)
float slewP = 0, slewR = 0;  // after slew limit
float outP = 0, outR = 0;    // after smoothing
float manP = 0, manR = 0;    // manual mode targets
float usP = 1500, usR = 1500;
bool satP = false, satR = false;
float nodOffset = 0;
uint32_t nodUntil = 0, softStartUntil = 0;
// housekeeping
uint32_t lastLoopUs = 0, loopCount = 0, loopHz = 0, statT = 0, saveAt = 0, telT = 0, rebootAt = 0, disconnectAt = 0;
uint32_t bootCount = 0, brownCount = 0;
esp_reset_reason_t rstReason;
bool firstBoot = false;

// ------------------------------------------------------------------------------------ small helpers
uint32_t crc32(const uint8_t *d, size_t n) {
  uint32_t c = 0xFFFFFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320 & (0 - (c & 1)));
  }
  return ~c;
}
float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}
float softDB(float e, float db) {  // dead band without a step at its edge
  if (e > db) return e - db;
  if (e < -db) return e + db;
  return 0.0f;
}

// ------------------------------------------------------------------------------------ SHA-256 (for the PIN check)
static const uint32_t SHA_K[64] = {
  0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
  0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
  0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
  0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
  0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
  0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
  0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
  0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};
static inline uint32_t ror32(uint32_t x, int r) {
  return (x >> r) | (x << (32 - r));
}
void shaBlock(Sha256 &s, const uint8_t *p) {
  uint32_t w[64];
  for (int i = 0; i < 16; i++) w[i] = ((uint32_t)p[4 * i] << 24) | ((uint32_t)p[4 * i + 1] << 16) | ((uint32_t)p[4 * i + 2] << 8) | p[4 * i + 3];
  for (int i = 16; i < 64; i++) {
    uint32_t s0 = ror32(w[i - 15], 7) ^ ror32(w[i - 15], 18) ^ (w[i - 15] >> 3);
    uint32_t s1 = ror32(w[i - 2], 17) ^ ror32(w[i - 2], 19) ^ (w[i - 2] >> 10);
    w[i] = w[i - 16] + s0 + w[i - 7] + s1;
  }
  uint32_t a = s.h[0], b = s.h[1], c = s.h[2], d = s.h[3], e = s.h[4], f = s.h[5], g = s.h[6], h = s.h[7];
  for (int i = 0; i < 64; i++) {
    uint32_t t1 = h + (ror32(e, 6) ^ ror32(e, 11) ^ ror32(e, 25)) + ((e & f) ^ (~e & g)) + SHA_K[i] + w[i];
    uint32_t t2 = (ror32(a, 2) ^ ror32(a, 13) ^ ror32(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
    h = g;
    g = f;
    f = e;
    e = d + t1;
    d = c;
    c = b;
    b = a;
    a = t1 + t2;
  }
  s.h[0] += a;
  s.h[1] += b;
  s.h[2] += c;
  s.h[3] += d;
  s.h[4] += e;
  s.h[5] += f;
  s.h[6] += g;
  s.h[7] += h;
}
void shaInit(Sha256 &s) {
  static const uint32_t H0[8] = { 0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19 };
  memcpy(s.h, H0, sizeof(H0));
  s.len = 0;
  s.n = 0;
}
void shaUpdate(Sha256 &s, const uint8_t *d, size_t n) {
  for (size_t i = 0; i < n; i++) {
    s.buf[s.n++] = d[i];
    s.len++;
    if (s.n == 64) {
      shaBlock(s, s.buf);
      s.n = 0;
    }
  }
}
void shaFinalHex(Sha256 &s, char out[65]) {
  uint64_t bits = s.len * 8;
  uint8_t pad = 0x80;
  shaUpdate(s, &pad, 1);
  uint8_t z = 0;
  while (s.n != 56) shaUpdate(s, &z, 1);
  uint8_t lb[8];
  for (int i = 0; i < 8; i++) lb[i] = (uint8_t)(bits >> (56 - 8 * i));
  shaUpdate(s, lb, 8);
  for (int i = 0; i < 8; i++) sprintf(out + 8 * i, "%08x", (unsigned)s.h[i]);
  out[64] = 0;
}
// hash = SHA256( nonce + ":" + pin ), lowercase hex
void authHash(const char *nonce, const char *pin, char out[65]) {
  Sha256 s;
  shaInit(s);
  shaUpdate(s, (const uint8_t *)nonce, strlen(nonce));
  shaUpdate(s, (const uint8_t *)":", 1);
  shaUpdate(s, (const uint8_t *)pin, strlen(pin));
  shaFinalHex(s, out);
}

// ------------------------------------------------------------------------------------ config load / save
void cfgDefaults(Config &c, bool keepIdentity) {
  char name[24], pin[16];
  memcpy(name, c.name, sizeof(name));
  memcpy(pin, c.pin, sizeof(pin));
  memset(&c, 0, sizeof(c));
  c.magic = CFG_MAGIC;
  c.size = sizeof(Config);
  c.pitchSet = 35.0f;
  c.rollSet = 0.0f;
  c.kp = 0.6f;
  c.gain = 15.0f;
  c.rateFF = 0.8f;
  c.deadband = 0.2f;
  c.smooth = 0.03f;
  c.maxRate = 400.0f;
  c.accTau = 0.6f;
  c.dlpf = 3;
  c.autoBias = 1;
  c.bootGyro = 1;
  c.nod = 1;
  c.dirP = 1;
  c.dirR = 1;
  c.startMode = MODE_STAB;
  c.usDegP = 10.5f;
  c.usDegR = 10.5f;
  c.limP = 25.0f;
  c.limR = 25.0f;
  c.axF = 0;
  c.axL = 1;
  c.axU = 2;
  c.sgF = 1;
  c.sgL = 1;
  c.sgU = 1;
  c.telHz = 10;
  c.ledOn = 1;
  if (keepIdentity) {
    memcpy(c.name, name, sizeof(name));
    memcpy(c.pin, pin, sizeof(pin));
  } else {
    strncpy(c.name, DEFAULT_NAME, sizeof(c.name) - 1);
    strncpy(c.pin, DEFAULT_PIN, sizeof(c.pin) - 1);
  }
  c.name[sizeof(c.name) - 1] = 0;
  c.pin[sizeof(c.pin) - 1] = 0;
}
bool cfgLoad() {
  Config t;
  if (prefs.getBytesLength("cfg") != sizeof(Config)) return false;
  prefs.getBytes("cfg", &t, sizeof(t));
  if (t.magic != CFG_MAGIC || t.size != sizeof(Config)) return false;
  if (t.crc != crc32((const uint8_t *)&t, offsetof(Config, crc))) return false;
  t.name[sizeof(t.name) - 1] = 0;
  t.pin[sizeof(t.pin) - 1] = 0;
  if (t.axF < 0 || t.axF > 2 || t.axL < 0 || t.axL > 2 || t.axU < 0 || t.axU > 2) return false;
  if (t.startMode > MODE_OFF) t.startMode = MODE_STAB;
  cfg = t;
  return true;
}
void cfgSave() {
  cfg.crc = crc32((const uint8_t *)&cfg, offsetof(Config, crc));
  prefs.putBytes("cfg", &cfg, sizeof(cfg));
  saveAt = 0;
}
void cfgSaveSoon() {
  saveAt = millis() + 2500;
  if (saveAt == 0) saveAt = 1;
}

// ------------------------------------------------------------------------------------ output (USB + Bluetooth)
BLEServer *bleServer = nullptr;
BLECharacteristic *bleTx = nullptr;
volatile bool bleConnected = false, bleJustConnected = false, bleJustDisconnected = false;
bool bleAuthed = false;
uint16_t bleMtu = 23;
char nonceHex[33] = "";
uint8_t authFails = 0;
uint32_t lockUntil = 0;
QueueHandle_t bleRxQ = nullptr;

static char txRing[6144];
static uint16_t txH = 0, txT = 0;
size_t txUsed() {
  return (size_t)((txH + sizeof(txRing) - txT) % sizeof(txRing));
}
size_t txFree() {
  return sizeof(txRing) - 1 - txUsed();
}
void txPush(const char *s, size_t n) {
  for (size_t i = 0; i < n; i++) {
    txRing[txH] = s[i];
    txH = (txH + 1) % sizeof(txRing);
  }
}
uint8_t allDest() {
  return TO_USB | ((bleConnected && bleAuthed) ? TO_BLE : 0);
}
void usbWrite(const char *s, size_t n) {
  if (!Serial) return;                                  // no computer listening
  if (Serial.availableForWrite() < (int)n + 1) return;  // never block the control loop
  Serial.write((const uint8_t *)s, n);
  Serial.write((uint8_t)'\n');
}
// returns false if the Bluetooth part had to be dropped (buffer full)
bool sendTo(uint8_t dest, const char *s) {
  size_t n = strlen(s);
  bool ok = true;
  if (dest & TO_USB) usbWrite(s, n);
  if ((dest & TO_BLE) && bleConnected) {
    if (txFree() >= n + 1) {
      txPush(s, n);
      txPush("\n", 1);
    } else ok = false;
  }
  return ok;
}
static char fmtBuf[1100];
bool sendf(uint8_t dest, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(fmtBuf, sizeof(fmtBuf), fmt, ap);
  va_end(ap);
  return sendTo(dest, fmtBuf);
}

// event log (last 16), also sent live
Ev evLog[16];
uint8_t evN = 0, evHead = 0;
void event(const char *fmt, ...) {
  Ev &e = evLog[evHead];
  e.t = millis();
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(e.s, sizeof(e.s), fmt, ap);
  va_end(ap);
  evHead = (evHead + 1) % 16;
  if (evN < 16) evN++;
  sendf(allDest(), "EV %lu %s", (unsigned long)e.t, e.s);
}

// ------------------------------------------------------------------------------------ servos (LEDC 50 Hz)
void pwmAttach(int pin) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, 50, 14);
#else
  ledcSetup(pin == PIN_SERVO_A ? 0 : 1, 50, 14);
  ledcAttachPin(pin, pin == PIN_SERVO_A ? 0 : 1);
#endif
}
void pwmWriteUs(int pin, float us) {  // us <= 0: no pulses (servo limp)
  uint32_t duty = 0;
  if (us > 0) {
    us = clampf(us, 500.0f, 2500.0f);
    duty = (uint32_t)(us * 16383.0f / 20000.0f + 0.5f);
  }
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(pin == PIN_SERVO_A ? 0 : 1, duty);
#endif
}
int pinPitch() {
  return cfg.swapServo ? PIN_SERVO_B : PIN_SERVO_A;
}
int pinRoll() {
  return cfg.swapServo ? PIN_SERVO_A : PIN_SERVO_B;
}

// ------------------------------------------------------------------------------------ MPU6050
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool mpuReadBytes(uint8_t reg, uint8_t *buf, int n) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if ((int)Wire.requestFrom((int)imuAddr, n) != n) return false;
  for (int i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}
bool mpuInit() {
  const uint8_t addrs[2] = { 0x68, 0x69 };
  bool found = false;
  for (int i = 0; i < 2 && !found; i++) {
    imuAddr = addrs[i];
    uint8_t w = 0;
    if (mpuReadBytes(0x75, &w, 1)) {
      imuWho = w;
      found = true;
    }
  }
  if (!found) {
    imuAddr = 0x68;
    return false;
  }
  bool ok = mpuWrite(0x6B, 0x01);    // wake up, gyro PLL clock
  ok &= mpuWrite(0x1A, cfg.dlpf & 7);  // digital low-pass filter
  ok &= mpuWrite(0x1B, 0x08);         // gyro +-500 deg/s
  ok &= mpuWrite(0x1C, 0x08);         // accel +-4 g
  ok &= mpuWrite(0x19, 4);            // 200 Hz sample rate
  return ok;
}
bool mpuRead() {
  uint8_t b[14];
  if (!mpuReadBytes(0x3B, b, 14)) {
    i2cErr++;
    return false;
  }
  bool allSame = true;
  for (int i = 1; i < 14; i++)
    if (b[i] != b[0]) allSame = false;
  if (allSame) {  // 00.. or FF..: sensor not really answering
    i2cErr++;
    return false;
  }
  for (int k = 0; k < 3; k++) {
    int16_t a = (int16_t)((b[2 * k] << 8) | b[2 * k + 1]);
    int16_t g = (int16_t)((b[8 + 2 * k] << 8) | b[9 + 2 * k]);
    accRaw[k] = a / 8192.0f;
    gyrUncal[k] = g / 65.5f;
    gyrRaw[k] = gyrUncal[k] - cfg.bias[k];
  }
  return true;
}

// ------------------------------------------------------------------------------------ attitude
// camera frame: F = forward (optical axis), L = left, U = up.
// pitch (theta) = nose DOWN positive, roll (phi) = right (USB) side DOWN positive. ZYX Euler, roll innermost.
void toCam(const float *v, float &f, float &l, float &u) {
  f = cfg.sgF * v[cfg.axF];
  l = cfg.sgL * v[cfg.axL];
  u = cfg.sgU * v[cfg.axU];
}
void accAngles(const float *a, float &th, float &ph) {
  float f, l, u;
  toCam(a, f, l, u);
  th = atan2f(-f, sqrtf(l * l + u * u)) * RAD_TO_DEG;
  ph = atan2f(l, u) * RAD_TO_DEG;
}
void attitudeReset() {
  accAngles(accRaw, thRaw, phRaw);
  pitchMeas = thRaw - cfg.lvlP;
  rollMeas = phRaw - cfg.lvlR;
}
void estimate(float dt) {
  float aF, aL, aU, gF, gL, gU;
  toCam(accRaw, aF, aL, aU);
  toCam(gyrRaw, gF, gL, gU);
  float ph = phRaw * DEG_TO_RAD, th = thRaw * DEG_TO_RAD;
  float c = cosf(th);
  if (fabsf(c) < 0.2f) c = (c < 0) ? -0.2f : 0.2f;
  float tanT = sinf(th) / c;
  float phDot = gF + tanT * (gL * sinf(ph) + gU * cosf(ph));
  float thDot = gL * cosf(ph) - gU * sinf(ph);
  thRaw += thDot * dt;
  phRaw += phDot * dt;
  // accelerometer correction, trusted less while walking/shaking (|a| far from 1 g)
  float an = sqrtf(aF * aF + aL * aL + aU * aU);
  float w = clampf(1.0f - fabsf(an - 1.0f) / 0.15f, 0.0f, 1.0f);
  accCount++;
  if (w < 0.5f) accRejCount++;
  if (an > 0.3f && w > 0.0f) {
    float thA = atan2f(-aF, sqrtf(aL * aL + aU * aU)) * RAD_TO_DEG;
    float phA = atan2f(aL, aU) * RAD_TO_DEG;
    float k = w * dt / (cfg.accTau + dt);
    thRaw += (thA - thRaw) * k;
    phRaw += wrap180(phA - phRaw) * k;
  }
  phRaw = wrap180(phRaw);
  thRaw = clampf(thRaw, -89.0f, 89.0f);
  pitchRate = thDot;
  rollRate = phDot;
  pitchMeas = thRaw - cfg.lvlP;
  rollMeas = phRaw - cfg.lvlR;
  // stillness (used by calibration and slow gyro-bias tracking)
  float gm = sqrtf(gF * gF + gL * gL + gU * gU);
  if (gm < 1.5f && fabsf(an - 1.0f) < 0.05f) stillTime += dt;
  else stillTime = 0;
  isStill = stillTime > 1.0f;
}

// ------------------------------------------------------------------------------------ calibration
bool calFreeze() {
  return cal.kind == CAL_LEVEL;
}

void calStart(CalKind k, uint8_t src) {
  cal = Cal();
  cal.kind = k;
  cal.src = src;
  cal.t0 = millis();
  sendf(allDest() | src, "CAL START %s", CAL_NAMES[k]);
}
void calFail(const char *why) {
  sendf(allDest() | cal.src, "CAL FAIL %s %s", CAL_NAMES[cal.kind], why);
  event("Kalibrasi %s gagal: %s", CAL_NAMES[cal.kind], why);
  cal.kind = CAL_IDLE;
  attitudeReset();
}
void calPhase(uint8_t p) {
  cal.phase = p;
  cal.t0 = millis();
  cal.n = 0;
  for (int k = 0; k < 3; k++) cal.s[k] = cal.ss[k] = cal.a[k] = 0;
  cal.gMax = 0;
}
float vdot(const float *a, const float *b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
float vnorm(const float *a) {
  return sqrtf(vdot(a, a));
}
int argmaxAbs(const float *v, int skip) {
  int best = -1;
  for (int i = 0; i < 3; i++) {
    if (i == skip) continue;
    if (best < 0 || fabsf(v[i]) > fabsf(v[best])) best = i;
  }
  return best;
}
const char AXN[3] = { 'X', 'Y', 'Z' };
void axesString(char *out, int8_t aF, int8_t sF, int8_t aL, int8_t sL, int8_t aU, int8_t sU) {
  sprintf(out, "%c%c%c%c%c%c", sF > 0 ? '+' : '-', AXN[aF], sL > 0 ? '+' : '-', AXN[aL], sU > 0 ? '+' : '-', AXN[aU]);
}

// Auto calibration. Gimbal upright as when worn (or standing on its clip), camera looking down, still.
// 1) gyro bias + gravity direction, 2) each servo moves +150 us and back: gyro gives the rotation axis,
// accelerometer gives the angle. Result: IMU axis mapping, servo cables swapped or not, direction and
// microseconds per degree of each servo.
enum { AP_SETTLE = 0, AP_GYRO, AP_BEFORE, AP_OUT, AP_HOLD, AP_BACK };
void calAutoSolve() {
  float m[2], n[2][3], c[2];
  for (int s = 0; s < 2; s++) {
    m[s] = vnorm(cal.rot[s]);
    if (m[s] < 4.0f) {
      static char msg[96];
      snprintf(msg, sizeof(msg), "Servo di GPIO%d tidak bergerak (%.1f deg). Cek kabel sinyal/5V/G dan horn.", s == 0 ? 3 : 4, m[s]);
      calFail(msg);
      return;
    }
    for (int k = 0; k < 3; k++) n[s][k] = cal.rot[s][k] / m[s];
    c[s] = fabsf(vdot(n[s], cal.up));
  }
  // roll servo turns the camera about its optical axis, which points ~35 deg down: its axis is closer to vertical
  if (fabsf(c[1] - c[0]) < 0.2f) {
    calFail("Posisi tidak jelas. Tegakkan dudukan seperti dipakai (kamera menunduk) lalu ulangi.");
    return;
  }
  int rollS = (c[1] >= c[0]) ? 1 : 0;
  int pitchS = 1 - rollS;
  int8_t newSwap = (rollS == 0) ? 1 : 0;
  // F axis = roll servo rotation axis, pointing down-forward (F . up < 0)
  int iF = argmaxAbs(n[rollS], -1);
  if (fabsf(n[rollS][iF]) < 0.8f) {
    calFail("IMU tidak sejajar dengan kamera (miring) atau dudukan bergoyang.");
    return;
  }
  if (fabsf(cal.up[iF]) < 0.12f) {
    calFail("Kamera harus menunduk minimal 10 derajat saat kalibrasi.");
    return;
  }
  int8_t sF = (cal.up[iF] > 0) ? -1 : 1;
  // U axis = what is left of 'up' after removing F
  float w[3] = { cal.up[0], cal.up[1], cal.up[2] };
  w[iF] = 0;
  int iU = argmaxAbs(w, iF);
  if (fabsf(w[iU]) < 0.5f) {
    calFail("Arah atas IMU tidak jelas. Tegakkan dudukan lalu ulangi.");
    return;
  }
  int8_t sU = (w[iU] > 0) ? 1 : -1;
  // L = U x F
  float Fv[3] = { 0, 0, 0 }, Uv[3] = { 0, 0, 0 }, Lv[3];
  Fv[iF] = sF;
  Uv[iU] = sU;
  Lv[0] = Uv[1] * Fv[2] - Uv[2] * Fv[1];
  Lv[1] = Uv[2] * Fv[0] - Uv[0] * Fv[2];
  Lv[2] = Uv[0] * Fv[1] - Uv[1] * Fv[0];
  int iL = 3 - iF - iU;
  int8_t sL = (Lv[iL] > 0) ? 1 : -1;
  float pitchAlign = fabsf(n[pitchS][iL]);
  // angles before/after each move with the NEW mapping
  int8_t oF = cfg.axF, oL = cfg.axL, oU = cfg.axU, osF = cfg.sgF, osL = cfg.sgL, osU = cfg.sgU;
  cfg.axF = iF;
  cfg.axL = iL;
  cfg.axU = iU;
  cfg.sgF = sF;
  cfg.sgL = sL;
  cfg.sgU = sU;
  float th0, ph0, th1, ph1, th2, ph2, th3, ph3;
  accAngles(cal.aBefore[pitchS], th0, ph0);
  accAngles(cal.aAfter[pitchS], th1, ph1);
  accAngles(cal.aBefore[rollS], th2, ph2);
  accAngles(cal.aAfter[rollS], th3, ph3);
  float dTh = th1 - th0, dPh = wrap180(ph3 - ph2);
  float usDegP = CAL_DUS / fmaxf(fabsf(dTh), 0.01f), usDegR = CAL_DUS / fmaxf(fabsf(dPh), 0.01f);
  if (usDegP < 6.0f || usDegP > 16.0f || usDegR < 6.0f || usDegR > 16.0f) {
    cfg.axF = oF;
    cfg.axL = oL;
    cfg.axU = oU;
    cfg.sgF = osF;
    cfg.sgL = osL;
    cfg.sgU = osU;
    static char msg[110];
    snprintf(msg, sizeof(msg), "Gerak servo tidak wajar (nunduk %.1f, kiri-kanan %.1f deg per 150us). Horn tersangkut / daya kurang?", dTh, dPh);
    calFail(msg);
    return;
  }
  bool mapChanged = (oF != iF || oL != iL || oU != iU || osF != sF || osL != sL || osU != sU);
  if (newSwap != cfg.swapServo) {  // trims belong to the physical servo
    int16_t t = cfg.trimP;
    cfg.trimP = cfg.trimR;
    cfg.trimR = t;
  }
  cfg.swapServo = newSwap;
  cfg.dirP = dTh > 0 ? 1 : -1;
  cfg.dirR = dPh > 0 ? 1 : -1;
  cfg.usDegP = usDegP;
  cfg.usDegR = usDegR;
  cfg.calOK |= 1 | 4;
  if (mode == MODE_CENTER && cfg.startMode == MODE_STAB) mode = MODE_STAB;  // calibrated: start stabilising
  if (mapChanged) {
    cfg.lvlP = 0;
    cfg.lvlR = 0;
    cfg.calOK &= ~2;
  }
  cfgSave();
  attitudeReset();
  cmdP = cmdR = slewP = slewR = outP = outR = 0;
  char ax[8];
  axesString(ax, cfg.axF, cfg.sgF, cfg.axL, cfg.sgL, cfg.axU, cfg.sgU);
  sendf(allDest() | cal.src,
        "CAL DONE auto {\"axes\":\"%s\",\"swap\":%d,\"dirp\":%d,\"dirr\":%d,\"usdegp\":%.2f,\"usdegr\":%.2f,"
        "\"gyrop\":%.1f,\"gyror\":%.1f,\"accp\":%.1f,\"accr\":%.1f,\"align\":%.2f,\"bias\":[%.2f,%.2f,%.2f],\"levelreset\":%d}",
        ax, cfg.swapServo, cfg.dirP, cfg.dirR, cfg.usDegP, cfg.usDegR, m[pitchS], m[rollS], dTh, dPh, pitchAlign,
        cfg.bias[0], cfg.bias[1], cfg.bias[2], mapChanged ? 1 : 0);
  event("Kalibrasi otomatis OK: sumbu %s, servo %s, %.1f/%.1f us/deg", ax, cfg.swapServo ? "TERTUKAR (dibetulkan)" : "benar",
        cfg.usDegP, cfg.usDegR);
  cal.kind = CAL_IDLE;
}

void calUpdate(float dt) {
  if (cal.kind == CAL_IDLE) return;
  uint32_t el = millis() - cal.t0;
  float gm = sqrtf(gyrRaw[0] * gyrRaw[0] + gyrRaw[1] * gyrRaw[1] + gyrRaw[2] * gyrRaw[2]);

  if (cal.kind == CAL_GYRO) {  // keep still 2 s
    for (int k = 0; k < 3; k++) {
      cal.s[k] += gyrUncal[k];
      cal.ss[k] += gyrUncal[k] * gyrUncal[k];
    }
    cal.n++;
    if (el >= 2000) {
      float sd = 0;
      for (int k = 0; k < 3; k++) {
        float mean = cal.s[k] / cal.n;
        sd = fmaxf(sd, sqrtf(fmaxf(cal.ss[k] / cal.n - mean * mean, 0)));
      }
      if (sd > 0.6f) {
        calFail("Gimbal bergerak. Letakkan diam lalu ulangi.");
        return;
      }
      for (int k = 0; k < 3; k++) cfg.bias[k] = cal.s[k] / cal.n;
      cfg.calOK |= 4;
      cfgSave();
      sendf(allDest() | cal.src, "CAL DONE gyro {\"bias\":[%.3f,%.3f,%.3f],\"noise\":%.3f}", cfg.bias[0], cfg.bias[1], cfg.bias[2], sd);
      event("Kalibrasi gyro OK (noise %.2f deg/s)", sd);
      cal.kind = CAL_IDLE;
    }
    return;
  }

  if (cal.kind == CAL_LEVEL) {  // outputs frozen; average 1.2 s after 0.3 s settle
    if (el < 300) return;
    cal.thSum += thRaw;
    cal.phSum += phRaw;
    cal.n++;
    cal.gMax = fmaxf(cal.gMax, gm);
    if (el >= 1500) {
      if (cal.gMax > 4.0f) {
        calFail("Kamera bergerak saat diukur. Tahan diam lalu ulangi.");
        return;
      }
      if (cal.useP) cfg.lvlP = cal.thSum / cal.n - cal.refP;
      if (cal.useR) cfg.lvlR = cal.phSum / cal.n - cal.refR;
      cfg.calOK |= 2;
      cfgSave();
      attitudeReset();
      sendf(allDest() | cal.src, "CAL DONE level {\"lvlp\":%.2f,\"lvlr\":%.2f}", cfg.lvlP, cfg.lvlR);
      event("Kalibrasi datar OK: koreksi nunduk %.2f, kiri-kanan %.2f deg", cfg.lvlP, cfg.lvlR);
      cal.kind = CAL_IDLE;
    }
    return;
  }

  // ---- CAL_AUTO state machine
  float *uA = &cal.usA, *uB = &cal.usB;
  float *mv = (cal.servo == 0) ? uA : uB;
  switch (cal.phase) {
    case AP_SETTLE:
      *uA = *uB = 0;
      if (el >= 1200) calPhase(AP_GYRO);
      break;
    case AP_GYRO:
      for (int k = 0; k < 3; k++) {
        cal.s[k] += gyrUncal[k];
        cal.ss[k] += gyrUncal[k] * gyrUncal[k];
        cal.a[k] += accRaw[k];
      }
      cal.n++;
      if (el >= 2000) {
        float sd = 0;
        for (int k = 0; k < 3; k++) {
          float mean = cal.s[k] / cal.n;
          sd = fmaxf(sd, sqrtf(fmaxf(cal.ss[k] / cal.n - mean * mean, 0)));
        }
        if (sd > 0.6f) {
          calFail("Gimbal bergerak. Letakkan diam (tegak seperti dipakai) lalu ulangi.");
          return;
        }
        for (int k = 0; k < 3; k++) {
          cfg.bias[k] = cal.s[k] / cal.n;
          cal.up[k] = cal.a[k] / cal.n;
        }
        float un = vnorm(cal.up);
        if (un < 0.85f || un > 1.15f) {
          calFail("Akselerometer tidak wajar (cek IMU).");
          return;
        }
        for (int k = 0; k < 3; k++) cal.up[k] /= un;
        cal.servo = 0;
        sendf(allDest() | cal.src, "CAL STEP gyro ok");
        calPhase(AP_BEFORE);
      }
      break;
    case AP_BEFORE:  // settle 0.4 s, average accel 0.4 s
      *mv = 0;
      if (el >= 400) {
        for (int k = 0; k < 3; k++) cal.a[k] += accRaw[k];
        cal.n++;
      }
      if (el >= 800) {
        for (int k = 0; k < 3; k++) {
          cal.aBefore[cal.servo][k] = cal.a[k] / cal.n;
          cal.rot[cal.servo][k] = 0;
        }
        sendf(allDest() | cal.src, "CAL STEP servo%d move", cal.servo == 0 ? 3 : 4);
        calPhase(AP_OUT);
      }
      break;
    case AP_OUT:  // ramp to +150 us in 0.6 s, integrate gyro
      *mv = CAL_DUS * fminf(el / 600.0f, 1.0f);
      for (int k = 0; k < 3; k++) cal.rot[cal.servo][k] += gyrRaw[k] * dt;
      if (el >= 600) calPhase(AP_HOLD);
      break;
    case AP_HOLD:  // keep integrating 0.4 s, then average accel 0.4 s
      *mv = CAL_DUS;
      if (el < 400) {
        for (int k = 0; k < 3; k++) cal.rot[cal.servo][k] += gyrRaw[k] * dt;
      } else {
        for (int k = 0; k < 3; k++) cal.a[k] += accRaw[k];
        cal.n++;
      }
      if (el >= 800) {
        for (int k = 0; k < 3; k++) cal.aAfter[cal.servo][k] = cal.a[k] / cal.n;
        calPhase(AP_BACK);
      }
      break;
    case AP_BACK:  // ramp back in 0.6 s, settle 0.3 s
      *mv = CAL_DUS * (1.0f - fminf(el / 600.0f, 1.0f));
      if (el >= 900) {
        if (cal.servo == 0) {
          cal.servo = 1;
          calPhase(AP_BEFORE);
        } else {
          calAutoSolve();
        }
      }
      break;
  }
}

// ------------------------------------------------------------------------------------ step test (response capture)
Sample capBuf[CAP_N];
int capI = 0, capDump = -1;
uint8_t capState = 0, capAxis = 0, capSrc = TO_USB;  // 0 idle, 1 capturing, 2 dumping
float capAmp = 0, stepOffP = 0, stepOffR = 0;
uint32_t capT0 = 0;
int capTick = 0;
void stepStart(uint8_t axis, float amp, uint8_t src) {
  capAxis = axis;
  capAmp = amp;
  capI = 0;
  capTick = 0;
  capState = 1;
  capSrc = src;
  capT0 = millis();
  sendf(allDest() | src, "STEP START %c %.1f", axis ? 'r' : 'p', amp);
}
void stepUpdate() {
  if (capState == 1) {
    if (++capTick % 2) return;  // 100 Hz
    uint32_t t = millis() - capT0;
    float off = (t >= 500 && t < 2000) ? capAmp : 0.0f;
    stepOffP = capAxis == 0 ? off : 0;
    stepOffR = capAxis == 1 ? off : 0;
    Sample &s = capBuf[capI];
    s.t = (uint16_t)t;
    s.sp = (int16_t)lroundf((cfg.pitchSet + stepOffP) * 100);
    s.p = (int16_t)lroundf(pitchMeas * 100);
    s.sr = (int16_t)lroundf((cfg.rollSet + stepOffR) * 100);
    s.r = (int16_t)lroundf(rollMeas * 100);
    s.op = (int16_t)lroundf(outP * 100);
    s.orr = (int16_t)lroundf(outR * 100);
    if (++capI >= CAP_N) {
      stepOffP = stepOffR = 0;
      capState = 2;
      capDump = -1;
    }
  } else if (capState == 2) {  // dump a few lines per loop without flooding Bluetooth
    uint8_t dest = allDest() | capSrc;
    for (int k = 0; k < 4; k++) {
      if ((dest & TO_BLE) && bleConnected && txFree() < 600) return;
      if (capDump < 0) {
        sendf(dest, "CAPBEGIN %c %.1f %d", capAxis ? 'r' : 'p', capAmp, CAP_N);
        capDump = 0;
        continue;
      }
      if (capDump >= CAP_N) {
        sendf(dest, "CAPEND");
        capState = 0;
        return;
      }
      Sample &s = capBuf[capDump];
      sendf(dest, "C,%d,%u,%d,%d,%d,%d,%d,%d", capDump, s.t, s.sp, s.p, s.sr, s.r, s.op, s.orr);
      capDump++;
    }
  }
}

// ------------------------------------------------------------------------------------ control
void control(float dt) {
  if (cal.kind == CAL_AUTO) {  // calibration drives the two pins directly
    float tA = cfg.swapServo ? cfg.trimR : cfg.trimP;
    float tB = cfg.swapServo ? cfg.trimP : cfg.trimR;
    pwmWriteUs(PIN_SERVO_A, 1500 + tA + cal.usA);
    pwmWriteUs(PIN_SERVO_B, 1500 + tB + cal.usB);
    return;
  }
  if (calFreeze()) return;  // level calibration: hold the outputs exactly where they are
  float limP = cfg.limP, limR = cfg.limR;
  float tgtP = 0, tgtR = 0;
  satP = satR = false;
  if (nodUntil && (int32_t)(millis() - nodUntil) >= 0) {
    nodUntil = 0;
    nodOffset = 0;
  }
  if (mode == MODE_STAB && imuOK && cal.kind == CAL_IDLE) {
    float eP = softDB((cfg.pitchSet + nodOffset + stepOffP) - pitchMeas, cfg.deadband);
    float eR = softDB((cfg.rollSet + stepOffR) - rollMeas, cfg.deadband);
    // PID on a position servo: I (gain) moves the servo until the camera is exactly on target,
    // P (kp) reacts immediately, D (rateFF) damps fast motion using the gyro rate.
    cmdP += (cfg.gain * eP - cfg.rateFF * pitchRate) * dt;
    cmdR += (cfg.gain * eR - cfg.rateFF * rollRate) * dt;
    cmdP = clampf(cmdP, -limP, limP);  // anti wind-up
    cmdR = clampf(cmdR, -limR, limR);
    float totP = cmdP + cfg.kp * eP, totR = cmdR + cfg.kp * eR;
    satP = totP >= limP || totP <= -limP;
    satR = totR >= limR || totR <= -limR;
    tgtP = clampf(totP, -limP, limP);
    tgtR = clampf(totR, -limR, limR);
  } else if (mode == MODE_MANUAL) {
    tgtP = clampf(manP, -limP, limP);
    tgtR = clampf(manR, -limR, limR);
    cmdP = tgtP;
    cmdR = tgtR;
  } else if (mode == MODE_STAB) {  // stabilising but IMU lost / gyro calibration: hold where we are
    tgtP = cmdP;
    tgtR = cmdR;
  } else {  // centre / off
    cmdP = cmdR = 0;
  }
  float rate = cfg.maxRate;
  if ((int32_t)(softStartUntil - millis()) > 0) rate = fminf(rate, 60.0f);
  float step = rate * dt;
  slewP += clampf(tgtP - slewP, -step, step);
  slewR += clampf(tgtR - slewR, -step, step);
  float k = dt / (cfg.smooth + dt);
  outP += (slewP - outP) * k;
  outR += (slewR - outR) * k;
  usP = 1500 + cfg.trimP + cfg.dirP * outP * cfg.usDegP;
  usR = 1500 + cfg.trimR + cfg.dirR * outR * cfg.usDegR;
  if (mode == MODE_OFF) {
    pwmWriteUs(pinPitch(), 0);
    pwmWriteUs(pinRoll(), 0);
  } else {
    pwmWriteUs(pinPitch(), usP);
    pwmWriteUs(pinRoll(), usR);
  }
}

// ------------------------------------------------------------------------------------ messages
const char *resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "crash";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_EXT: return "external";
    default: return "other";
  }
}
void sendHello(uint8_t dest) {
  bool needAuth = (dest == TO_BLE) && !bleAuthed;
  long lock = 0;
  if ((int32_t)(lockUntil - millis()) > 0) lock = (lockUntil - millis()) / 1000 + 1;
  sendf(dest, "HELLO {\"fw\":\"%s\",\"name\":\"%s\",\"auth\":%d,\"nonce\":\"%s\",\"lock\":%ld}", FW_VERSION, cfg.name,
        needAuth ? 1 : 0, nonceHex, lock);
}
void sendCfg(uint8_t dest) {
  char ax[8];
  axesString(ax, cfg.axF, cfg.sgF, cfg.axL, cfg.sgL, cfg.axU, cfg.sgU);
  sendf(dest,
        "CFG {\"pitch\":%.2f,\"roll\":%.2f,\"kp\":%.2f,\"gain\":%.2f,\"ff\":%.2f,\"db\":%.2f,\"smooth\":%.3f,\"maxrate\":%.0f,"
        "\"acctau\":%.2f,\"dlpf\":%u,\"autobias\":%u,\"bootgyro\":%u,\"nod\":%u,\"trimp\":%d,\"trimr\":%d,\"dirp\":%d,"
        "\"dirr\":%d,\"swap\":%d,\"usdegp\":%.2f,\"usdegr\":%.2f,\"limp\":%.1f,\"limr\":%.1f,\"lvlp\":%.2f,\"lvlr\":%.2f,"
        "\"axes\":\"%s\",\"tel\":%d,\"led\":%d,\"startmode\":%d,\"name\":\"%s\",\"cal\":%u,\"defpin\":%d,\"mode\":\"%s\"}",
        cfg.pitchSet, cfg.rollSet, cfg.kp, cfg.gain, cfg.rateFF, cfg.deadband, cfg.smooth, cfg.maxRate, cfg.accTau, cfg.dlpf,
        cfg.autoBias, cfg.bootGyro, cfg.nod, cfg.trimP, cfg.trimR, cfg.dirP, cfg.dirR, cfg.swapServo, cfg.usDegP,
        cfg.usDegR, cfg.limP, cfg.limR, cfg.lvlP, cfg.lvlR, ax, cfg.telHz, cfg.ledOn, cfg.startMode, cfg.name, cfg.calOK,
        strcmp(cfg.pin, DEFAULT_PIN) == 0 ? 1 : 0, MODE_NAMES[mode]);
}
void sendStatus(uint8_t dest) {
  sendf(dest,
        "ST {\"fw\":\"%s\",\"up\":%lu,\"hz\":%lu,\"rst\":\"%s\",\"boots\":%lu,\"brown\":%lu,\"imu\":%d,\"who\":\"0x%02X\","
        "\"addr\":\"0x%02X\",\"i2cerr\":%lu,\"temp\":%.1f,\"bias\":[%.2f,%.2f,%.2f],\"heap\":%lu,\"ble\":%d,\"mtu\":%u,"
        "\"mode\":\"%s\",\"cal\":\"%s\",\"still\":%d,\"accrej\":%.0f,\"usp\":%.0f,\"usr\":%.0f,\"swap\":%d}",
        FW_VERSION, (unsigned long)(millis() / 1000), (unsigned long)loopHz, resetReasonStr(rstReason),
        (unsigned long)bootCount, (unsigned long)brownCount, imuOK ? 1 : 0, imuWho, imuAddr, (unsigned long)i2cErr,
        temperatureRead(), cfg.bias[0], cfg.bias[1], cfg.bias[2], (unsigned long)ESP.getFreeHeap(),
        bleConnected ? (bleAuthed ? 2 : 1) : 0, bleMtu, MODE_NAMES[mode], CAL_NAMES[cal.kind], isStill ? 1 : 0,
        accRejPct, usP, usR, cfg.swapServo);
}
void sendHelp(uint8_t dest) {
  static const char *lines[] = {
    "HELP perintah:",
    "HELP GET | STATUS | EVLOG | PING | HELLO",
    "HELP SET <kunci> <nilai>   (pitch roll kp gain ff db smooth maxrate acctau dlpf autobias bootgyro nod",
    "HELP     trimp trimr dirp dirr swap usdegp usdegr limp limr lvlp lvlr axes tel led startmode)",
    "HELP MODE stab|center|manual|off   MAN <nunduk> <kirikanan>",
    "HELP CAL auto | CAL gyro | CAL level [nunduk|x] [kirikanan|x] | CAL abort",
    "HELP STEP p|r <derajat>   SAVE   DEFAULTS tuning|all   PIN <baru>   NAME <nama>   REBOOT",
  };
  for (auto l : lines) sendTo(dest, l);
}

// ------------------------------------------------------------------------------------ SET
bool setParam(const char *key, const char *val, char *err, size_t errLen) {
  float v = atof(val);
  int iv = atoi(val);
#define F_(k, field, lo, hi) \
  if (!strcmp(key, k)) { \
    cfg.field = clampf(v, lo, hi); \
    return true; \
  }
#define I_(k, field, lo, hi) \
  if (!strcmp(key, k)) { \
    cfg.field = (int)clampf((float)iv, lo, hi); \
    return true; \
  }
  F_("pitch", pitchSet, 10.0f, 60.0f)
  F_("roll", rollSet, -15.0f, 15.0f)
  F_("kp", kp, 0.0f, 1.5f)
  F_("gain", gain, 0.5f, 30.0f)
  F_("ff", rateFF, 0.0f, 1.5f)
  F_("db", deadband, 0.0f, 2.0f)
  F_("smooth", smooth, 0.0f, 0.3f)
  F_("maxrate", maxRate, 30.0f, 800.0f)
  F_("acctau", accTau, 0.1f, 5.0f)
  I_("autobias", autoBias, 0, 1)
  I_("bootgyro", bootGyro, 0, 1)
  I_("nod", nod, 0, 1)
  I_("trimp", trimP, -400, 400)
  I_("trimr", trimR, -400, 400)
  I_("swap", swapServo, 0, 1)
  F_("usdegp", usDegP, 5.0f, 18.0f)
  F_("usdegr", usDegR, 5.0f, 18.0f)
  F_("limp", limP, 3.0f, 25.0f)
  F_("limr", limR, 3.0f, 25.0f)
  F_("lvlp", lvlP, -30.0f, 30.0f)
  F_("lvlr", lvlR, -30.0f, 30.0f)
  I_("tel", telHz, 0, 25)
  I_("led", ledOn, 0, 1)
  I_("startmode", startMode, 0, 3)
#undef F_
#undef I_
  if (!strcmp(key, "dlpf")) {
    cfg.dlpf = (uint8_t)clampf((float)iv, 0, 6);
    if (imuOK) mpuWrite(0x1A, cfg.dlpf);
    return true;
  }
  if (!strcmp(key, "dirp") || !strcmp(key, "dirr")) {
    int8_t d = (iv < 0) ? -1 : 1;
    if (key[3] == 'p') cfg.dirP = d;
    else cfg.dirR = d;
    return true;
  }
  if (!strcmp(key, "axes")) {  // e.g. +X+Y+Z  (forward, left, up)
    if (strlen(val) != 6) {
      snprintf(err, errLen, "format axes: +X+Y+Z (maju, kiri, atas)");
      return false;
    }
    int8_t a[3], s[3];
    for (int i = 0; i < 3; i++) {
      char sc = val[2 * i], ac = toupper(val[2 * i + 1]);
      if ((sc != '+' && sc != '-') || ac < 'X' || ac > 'Z') {
        snprintf(err, errLen, "format axes: +X+Y+Z");
        return false;
      }
      s[i] = sc == '+' ? 1 : -1;
      a[i] = ac - 'X';
    }
    if (a[0] == a[1] || a[0] == a[2] || a[1] == a[2]) {
      snprintf(err, errLen, "sumbu tidak boleh dobel");
      return false;
    }
    cfg.axF = a[0];
    cfg.axL = a[1];
    cfg.axU = a[2];
    cfg.sgF = s[0];
    cfg.sgL = s[1];
    cfg.sgU = s[2];
    attitudeReset();
    return true;
  }
  snprintf(err, errLen, "kunci tidak dikenal: %s", key);
  return false;
}

// ------------------------------------------------------------------------------------ command handler
void newNonce() {
  for (int i = 0; i < 16; i++) sprintf(nonceHex + 2 * i, "%02x", (unsigned)(esp_random() & 0xFF));
}
void handleAuth(const char *hex, uint8_t src) {
  if (src == TO_USB) {
    sendTo(src, "AUTH OK");
    return;
  }
  uint32_t now = millis();
  if ((int32_t)(lockUntil - now) > 0) {
    sendf(TO_BLE, "AUTH LOCKED %lu", (unsigned long)((lockUntil - now) / 1000 + 1));
    return;
  }
  char expect[65];
  authHash(nonceHex, cfg.pin, expect);
  if (hex && strlen(hex) == 64 && strcasecmp(hex, expect) == 0) {
    bleAuthed = true;
    authFails = 0;
    sendTo(TO_BLE, "AUTH OK");
    event("Bluetooth tersambung, sandi benar");
    if (cfg.nod && mode == MODE_STAB) {  // small nod: physical "connected" sign
      nodOffset = 4.0f;
      nodUntil = millis() + 400;
      if (nodUntil == 0) nodUntil = 1;
    }
  } else {
    authFails++;
    newNonce();
    if (authFails >= 5) {
      authFails = 0;
      lockUntil = now + 30000;
      sendTo(TO_BLE, "AUTH LOCKED 30");
      event("5x sandi salah lewat Bluetooth: dikunci 30 detik");
      disconnectAt = now + 400;
      if (disconnectAt == 0) disconnectAt = 1;
    } else {
      sendf(TO_BLE, "AUTH FAIL %d %s", 5 - authFails, nonceHex);
    }
  }
}

void handleLine(char *line, uint8_t src) {
  char *save = nullptr;
  char *cmd = strtok_r(line, " ", &save);
  if (!cmd) return;
  for (char *p = cmd; *p; p++) *p = toupper(*p);
  if (!strcmp(cmd, "HELLO")) {
    sendHello(src);
    return;
  }
  if (!strcmp(cmd, "AUTH")) {
    handleAuth(strtok_r(nullptr, " ", &save), src);
    return;
  }
  if (!strcmp(cmd, "PING")) {
    sendf(src, "PONG %lu", (unsigned long)millis());
    return;
  }
  bool authed = (src == TO_USB) || bleAuthed;
  if (!authed) {
    sendTo(src, "ERR AUTH");
    return;
  }
  char *a1 = strtok_r(nullptr, " ", &save);
  char *a2 = strtok_r(nullptr, " ", &save);
  char *a3 = strtok_r(nullptr, " ", &save);
  if (!strcmp(cmd, "GET")) {
    sendCfg(src);
  } else if (!strcmp(cmd, "STATUS")) {
    sendStatus(src);
  } else if (!strcmp(cmd, "HELP")) {
    sendHelp(src);
  } else if (!strcmp(cmd, "EVLOG")) {
    for (int i = 0; i < evN; i++) {
      Ev &e = evLog[(evHead + 16 - evN + i) % 16];
      sendf(src, "EV %lu %s", (unsigned long)e.t, e.s);
    }
    sendTo(src, "EVEND");
  } else if (!strcmp(cmd, "SET")) {
    if (!a1 || !a2) {
      sendTo(src, "ERR SET <kunci> <nilai>");
      return;
    }
    for (char *p = a1; *p; p++) *p = tolower(*p);
    char err[64];
    if (setParam(a1, a2, err, sizeof(err))) {
      cfgSaveSoon();
      sendCfg(allDest() | src);  // everyone sees the new value
    } else sendf(src, "ERR %s", err);
  } else if (!strcmp(cmd, "MODE")) {
    if (!a1) {
      sendTo(src, "ERR MODE stab|center|manual|off");
      return;
    }
    for (char *p = a1; *p; p++) *p = tolower(*p);
    int m = -1;
    for (int i = 0; i < 4; i++)
      if (!strcmp(a1, MODE_NAMES[i])) m = i;
    if (m < 0) {
      sendTo(src, "ERR MODE stab|center|manual|off");
      return;
    }
    if (cal.kind != CAL_IDLE) {
      sendTo(src, "ERR kalibrasi sedang berjalan");
      return;
    }
    if (m == MODE_MANUAL) {
      manP = outP;
      manR = outR;
    }
    if (m == MODE_STAB && mode != MODE_STAB) {
      cmdP = outP;
      cmdR = outR;
    }
    mode = (Mode)m;
    if (m == MODE_STAB && !(cfg.calOK & 1)) sendTo(src, "WARN belum dikalibrasi: arah servo/IMU bisa salah. Jalankan CAL auto.");
    sendCfg(allDest() | src);
  } else if (!strcmp(cmd, "MAN")) {
    if (!a1 || !a2) {
      sendTo(src, "ERR MAN <nunduk> <kirikanan>");
      return;
    }
    if (mode != MODE_MANUAL) {
      sendTo(src, "ERR pindah ke MODE manual dulu");
      return;
    }
    manP = clampf(atof(a1), -cfg.limP, cfg.limP);
    manR = clampf(atof(a2), -cfg.limR, cfg.limR);
    sendf(src, "OK MAN %.2f %.2f", manP, manR);
  } else if (!strcmp(cmd, "CAL")) {
    if (!a1) {
      sendTo(src, "ERR CAL auto|gyro|level|abort");
      return;
    }
    for (char *p = a1; *p; p++) *p = tolower(*p);
    if (!strcmp(a1, "abort")) {
      if (cal.kind != CAL_IDLE) calFail("dibatalkan");
      return;
    }
    if (cal.kind != CAL_IDLE || capState) {
      sendTo(src, "ERR kalibrasi/uji sedang berjalan");
      return;
    }
    if (!imuOK) {
      sendTo(src, "ERR IMU tidak terbaca, cek 4 kabel IMU");
      return;
    }
    if (!strcmp(a1, "gyro")) {
      calStart(CAL_GYRO, src);
    } else if (!strcmp(a1, "level")) {
      // CAL level <nunduk terbaca|x> <kiri-kanan terbaca|x>   (x = jangan ubah sumbu itu)
      calStart(CAL_LEVEL, src);
      cal.useP = !(a2 && tolower(a2[0]) == 'x');
      cal.useR = !(a3 && tolower(a3[0]) == 'x');
      cal.refP = (a2 && cal.useP) ? atof(a2) : 0.0f;
      cal.refR = (a3 && cal.useR) ? atof(a3) : 0.0f;
    } else if (!strcmp(a1, "auto")) {
      if (mode == MODE_OFF) {
        sendTo(src, "ERR servo mati (MODE off). Pilih mode lain dulu");
        return;
      }
      calStart(CAL_AUTO, src);
      calPhase(AP_SETTLE);
    } else sendTo(src, "ERR CAL auto|gyro|level|abort");
  } else if (!strcmp(cmd, "STEP")) {
    if (mode != MODE_STAB || !imuOK || cal.kind != CAL_IDLE || capState) {
      sendTo(src, "ERR uji respons hanya di mode stab, tanpa kalibrasi berjalan");
      return;
    }
    uint8_t ax = (a1 && tolower(a1[0]) == 'r') ? 1 : 0;
    float amp = a2 ? clampf(atof(a2), -10.0f, 10.0f) : 5.0f;
    if (fabsf(amp) < 0.5f) amp = 5.0f;
    stepStart(ax, amp, src);
  } else if (!strcmp(cmd, "SAVE")) {
    cfgSave();
    sendTo(src, "OK SAVE");
  } else if (!strcmp(cmd, "DEFAULTS")) {
    bool all = a1 && !strcasecmp(a1, "all");
    Config keep = cfg;
    cfgDefaults(cfg, true);
    if (!all) {  // tuning only: keep calibration + servo setup
      cfg.calOK = keep.calOK;
      cfg.startMode = keep.startMode;
      cfg.trimP = keep.trimP;
      cfg.trimR = keep.trimR;
      cfg.dirP = keep.dirP;
      cfg.dirR = keep.dirR;
      cfg.swapServo = keep.swapServo;
      cfg.usDegP = keep.usDegP;
      cfg.usDegR = keep.usDegR;
      cfg.axF = keep.axF;
      cfg.axL = keep.axL;
      cfg.axU = keep.axU;
      cfg.sgF = keep.sgF;
      cfg.sgL = keep.sgL;
      cfg.sgU = keep.sgU;
      cfg.lvlP = keep.lvlP;
      cfg.lvlR = keep.lvlR;
      memcpy(cfg.bias, keep.bias, sizeof(cfg.bias));
    }
    cfgSave();
    attitudeReset();
    event("Pengaturan dikembalikan ke bawaan (%s)", all ? "semua" : "tuning");
    sendCfg(allDest() | src);
  } else if (!strcmp(cmd, "FACTORY")) {
    if (src != TO_USB) {
      sendTo(src, "ERR FACTORY hanya lewat kabel USB");
      return;
    }
    cfgDefaults(cfg, false);
    cfgSave();
    sendTo(src, "OK FACTORY, restart...");
    rebootAt = millis() + 300;
  } else if (!strcmp(cmd, "PIN")) {
    if (!a1) {
      sendTo(src, "ERR PIN <sandi baru 4-12 huruf/angka>");
      return;
    }
    size_t n = strlen(a1);
    bool ok = n >= 4 && n <= 12;
    for (size_t i = 0; i < n && ok; i++)
      if (!isalnum((unsigned char)a1[i])) ok = false;
    if (!ok) {
      sendTo(src, "ERR sandi harus 4-12 huruf/angka");
      return;
    }
    strncpy(cfg.pin, a1, sizeof(cfg.pin) - 1);
    cfg.pin[sizeof(cfg.pin) - 1] = 0;
    cfgSave();
    sendTo(src, "OK PIN");
    event("Sandi Bluetooth diganti");
  } else if (!strcmp(cmd, "NAME")) {
    if (!a1) {
      sendTo(src, "ERR NAME <nama, maks 20, tanpa spasi pakai _>");
      return;
    }
    char nm[24];
    size_t j = 0;
    for (size_t i = 0; a1[i] && j < 20; i++) {
      char c = a1[i] == '_' ? ' ' : a1[i];
      if (isalnum((unsigned char)c) || c == ' ' || c == '-') nm[j++] = c;
    }
    nm[j] = 0;
    if (j < 2) {
      sendTo(src, "ERR nama terlalu pendek");
      return;
    }
    strcpy(cfg.name, nm);
    cfgSave();
    sendf(src, "OK NAME %s (berlaku setelah restart)", cfg.name);
  } else if (!strcmp(cmd, "REBOOT")) {
    if (saveAt) cfgSave();
    sendTo(src, "OK REBOOT");
    rebootAt = millis() + 300;
  } else {
    sendf(src, "ERR perintah tidak dikenal: %s (kirim HELP)", cmd);
  }
}

// ------------------------------------------------------------------------------------ Bluetooth
class SrvCB : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override {
    bleConnected = true;
    bleJustConnected = true;
  }
  void onDisconnect(BLEServer *s) override {
    bleConnected = false;
    bleJustDisconnected = true;
  }
};
class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    auto v = c->getValue();
    for (size_t i = 0; i < v.length(); i++) {
      char ch = v[i];
      if (bleRxQ) xQueueSend(bleRxQ, &ch, 0);
    }
  }
};
void bleInit() {
  bleRxQ = xQueueCreate(768, sizeof(char));
  BLEDevice::init(cfg.name);
  BLEDevice::setMTU(247);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new SrvCB());
  BLEService *svc = bleServer->createService(NUS_SERVICE);
  bleTx = svc->createCharacteristic(NUS_TX, BLECharacteristic::PROPERTY_NOTIFY);
#if defined(CONFIG_BLUEDROID_ENABLED)
  bleTx->addDescriptor(new BLE2902());
#endif
  BLECharacteristic *rx = svc->createCharacteristic(NUS_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCB());
  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  newNonce();
}
void bleHousekeeping() {
  if (bleJustConnected) {
    bleJustConnected = false;
    bleAuthed = false;
    bleMtu = 23;
    txH = txT = 0;
    newNonce();
  }
  if (bleJustDisconnected) {
    bleJustDisconnected = false;
    bool was = bleAuthed;
    bleAuthed = false;
    txH = txT = 0;
    BLEDevice::startAdvertising();
    if (was) event("Bluetooth terputus");
  }
  if (disconnectAt && (int32_t)(millis() - disconnectAt) >= 0) {
    disconnectAt = 0;
    if (bleConnected && bleServer) bleServer->disconnect(bleServer->getConnId());
  }
}
void blePump() {
  static uint32_t last = 0;
  if (!bleConnected || !bleTx) return;
  uint32_t now = micros();
  if ((uint32_t)(now - last) < 7500) return;
  last = now;
  size_t used = txUsed();
  if (!used) return;
  size_t chunk = (size_t)bleMtu - 3;
  if (chunk > 180) chunk = 180;
  if (chunk > used) chunk = used;
  uint8_t buf[184];
  for (size_t i = 0; i < chunk; i++) {
    buf[i] = (uint8_t)txRing[txT];
    txT = (txT + 1) % sizeof(txRing);
  }
  bleTx->setValue(buf, chunk);
  bleTx->notify();
}
void pollInputs() {
  static char ub[200], bb[200];
  static uint8_t un = 0, bn = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      ub[un] = 0;
      if (un) handleLine(ub, TO_USB);
      un = 0;
    } else if (un < sizeof(ub) - 1) ub[un++] = c;
  }
  char c;
  while (bleRxQ && xQueueReceive(bleRxQ, &c, 0) == pdTRUE) {
    if (c == '\r') continue;
    if (c == '\n') {
      bb[bn] = 0;
      if (bn) handleLine(bb, TO_BLE);
      bn = 0;
    } else if (bn < sizeof(bb) - 1) bb[bn++] = c;
  }
}

// ------------------------------------------------------------------------------------ LED + telemetry
void ledUpdate() {
  if (!cfg.ledOn) {
    digitalWrite(PIN_LED, HIGH);
    return;
  }
  uint32_t t = millis();
  bool on;
  if (!imuOK) on = (t / 125) % 2;
  else if (cal.kind != CAL_IDLE || capState) on = (t / 250) % 2;
  else if (bleConnected && bleAuthed) on = true;
  else if (bleConnected) on = (t / 500) % 2;
  else on = (t % 2000) < 60;
  digitalWrite(PIN_LED, on ? LOW : HIGH);
}
void telemetry() {
  if (!cfg.telHz || capState == 2) return;
  uint32_t now = millis();
  if ((uint32_t)(now - telT) < 1000u / cfg.telHz) return;
  telT = now;
  uint8_t dest = allDest();
  if ((dest & TO_BLE) && txUsed() > sizeof(txRing) / 3) dest &= ~TO_BLE;  // Bluetooth busy: skip one
  uint8_t flags = (imuOK ? 1 : 0) | (bleAuthed ? 2 : 0) | (isStill ? 4 : 0) | (satP ? 8 : 0) | (satR ? 16 : 0) |
                  (cal.kind != CAL_IDLE ? 32 : 0) | (accRejPct > 50 ? 64 : 0);
  sendf(dest, "T,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%u,%u", (unsigned long)now, pitchMeas, rollMeas,
        cfg.pitchSet + nodOffset + stepOffP, cfg.rollSet + stepOffR, outP, outR, pitchRate, rollRate, (unsigned)mode, flags);
}

// ------------------------------------------------------------------------------------ setup / loop
void bootGyro() {
  float s[3] = { 0, 0, 0 }, ss[3] = { 0, 0, 0 };
  int n = 0;
  for (int i = 0; i < 300; i++) {
    if (mpuRead()) {
      for (int k = 0; k < 3; k++) {
        s[k] += gyrUncal[k];
        ss[k] += gyrUncal[k] * gyrUncal[k];
      }
      n++;
    }
    delay(5);
  }
  if (n < 200) return;
  float sd = 0;
  for (int k = 0; k < 3; k++) {
    float m = s[k] / n;
    sd = fmaxf(sd, sqrtf(fmaxf(ss[k] / n - m * m, 0)));
  }
  if (sd < 0.6f) {
    for (int k = 0; k < 3; k++) cfg.bias[k] = s[k] / n;
    cfg.calOK |= 4;
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  Serial.begin(115200);
  prefs.begin("gimbal", false);
  if (!cfgLoad()) {
    memset(&cfg, 0, sizeof(cfg));
    cfgDefaults(cfg, false);
    cfgSave();
    firstBoot = true;
  }
  rstReason = esp_reset_reason();
  bootCount = prefs.getUInt("boots", 0) + 1;
  prefs.putUInt("boots", bootCount);
  brownCount = prefs.getUInt("brown", 0);
  if (rstReason == ESP_RST_BROWNOUT) prefs.putUInt("brown", ++brownCount);
  mode = (Mode)cfg.startMode;
  if (!(cfg.calOK & 1) && mode == MODE_STAB) mode = MODE_CENTER;  // not calibrated yet: hold the centre

  // servos one after the other, so their start-up current does not add up
  pwmAttach(PIN_SERVO_A);
  pwmAttach(PIN_SERVO_B);
  if (mode != MODE_OFF) {
    pwmWriteUs(pinPitch(), 1500 + cfg.trimP);
    delay(350);
    pwmWriteUs(pinRoll(), 1500 + cfg.trimR);
    delay(350);
  }

  Wire.begin(PIN_SDA, PIN_SCL, 100000);  // 100 kHz: robust over the long moving IMU wires
  Wire.setTimeOut(10);
  imuOK = mpuInit();
  bleInit();
  if (imuOK) {
    delay(100);
    if (cfg.bootGyro) bootGyro();
    for (int i = 0; i < 5; i++) {
      mpuRead();
      delay(5);
    }
    attitudeReset();
  }
  softStartUntil = millis() + 2500;
  event("Menyala (fw %s). Restart karena: %s. Restart tegangan turun total: %lu", FW_VERSION, resetReasonStr(rstReason),
        (unsigned long)brownCount);
  if (!imuOK) event("IMU tidak terbaca: cek VCC->3V3, GND->G, SDA->GPIO6, SCL->GPIO7");
  if (!(cfg.calOK & 1)) event("Belum dikalibrasi: servo ditahan di tengah. Jalankan Kalibrasi Otomatis.");
  lastLoopUs = micros();
  statT = millis();
}

void loop() {
  pollInputs();
  bleHousekeeping();
  blePump();
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastLoopUs) < LOOP_US) return;
  float dt = (nowUs - lastLoopUs) * 1e-6f;
  if (dt > 0.05f) dt = 0.05f;
  lastLoopUs = nowUs;
  loopCount++;

  if (mpuRead()) {
    if (!imuOK) {
      imuOK = true;
      attitudeReset();
      event("IMU terbaca lagi");
    }
    imuFailRun = 0;
    estimate(dt);
    if (cfg.autoBias && stillTime > 2.0f && mode == MODE_STAB && cal.kind == CAL_IDLE) {
      for (int k = 0; k < 3; k++) cfg.bias[k] += 0.0005f * gyrRaw[k];  // follow slow gyro drift
    }
  } else {
    imuFailRun++;
    if (imuFailRun == 20 && imuOK) {
      imuOK = false;
      if (cal.kind != CAL_IDLE) calFail("IMU berhenti menjawab");
      event("IMU ERROR: tidak menjawab. Servo ditahan. Cek 4 kabel IMU.");
    }
    if (!imuOK && imuFailRun % 200 == 0) {  // try again every second
      Wire.end();
      Wire.begin(PIN_SDA, PIN_SCL, 100000);
      Wire.setTimeOut(10);
      mpuInit();
    }
  }
  calUpdate(dt);
  stepUpdate();
  control(dt);
  ledUpdate();
  telemetry();

  uint32_t now = millis();
  if ((uint32_t)(now - statT) >= 1000) {
    loopHz = loopCount;
    loopCount = 0;
    accRejPct = accCount ? 100.0f * accRejCount / accCount : 0;
    accCount = accRejCount = 0;
    statT = now;
    if (bleConnected && bleServer) {
      uint16_t m = bleServer->getPeerMTU(bleServer->getConnId());
      bleMtu = (m >= 23 && m <= 517) ? m : 23;
    }
  }
  if (saveAt && (int32_t)(now - saveAt) >= 0) cfgSave();
  if (rebootAt && (int32_t)(now - rebootAt) >= 0) ESP.restart();
}
