/*
  ============================================================================================
  GIMBAL D435 (sabuk) - firmware v4.1
  ESP32-C3 Super Mini + MPU6050 (GY-521) + 2x MG90S 180 derajat
  ============================================================================================
  Fitur
   - Menstabilkan sumbu atas-bawah (pitch) dan sumbu kanan-kiri (roll). IMU menempel di bawah dudukan
     kamera, sehingga yang diukur adalah kamera itu sendiri (closed loop), ditambah umpan maju:
     gerak badan dihitung dari IMU dikurangi model servo, lalu langsung dilawan.
   - WiFi: gimbal menyajikan aplikasinya sendiri. Hubungkan gimbal ke hotspot ponsel, lalu buka di
     Safari/Chrome:  http://gimbal-d435.local   (atau alamat IP yang tampil di aplikasi / Serial Monitor)
     Pada pemakaian pertama (belum mengenal hotspot apa pun) gimbal memancarkan WiFi
     "Gimbal-D435-xxxx" (sandi gimbald435): hubungkan ponsel/iPad ke WiFi itu, buka http://192.168.4.1,
     lalu isi data hotspot di menu Perangkat. Beberapa ponsel/iPad dapat terhubung bersamaan; masing-
     masing memerlukan sandi (PIN, challenge-response SHA-256: sandi tidak pernah dikirim).
     Kabel USB tetap dapat dipakai.
   - Semua pengaturan tersimpan di flash dan tetap ada walau listrik dicabut.
   - Tanpa kalibrasi servo: arah, kabel, skala, dan kecepatan servo diatur sendiri di aplikasi.
     Kecepatan bawaan pelan (awet untuk MG90S); di atas 200 derajat/detik muncul peringatan.
   - Gerak halus: perintah servo mengikuti profil kecepatan dan percepatan terbatas (mulai pelan,
     melaju, mengerem tepat di target), sehingga tidak menyentak dan getaran cepat tidak diteruskan.
   - Kalibrasi datar (memakai waterpass / aplikasi waterpass di ponsel) dan kalibrasi gyro.
   - Uji respons (step test), log kejadian, penyebab restart (termasuk tegangan turun).
   - Servo menyala bergantian dan bergerak pelan di awal (lonjakan arus kecil).
   - LED biru (GPIO8): berkedip dua kali tiap 2 s = WiFi gimbal aktif dan menunggu diatur,
     kedip singkat tiap 1 s = mencari hotspot, kedip singkat tiap 2 s = siap (terhubung ke hotspot),
     berkedip 1 Hz = aplikasi terhubung tetapi sandi belum dimasukkan, menyala terus = aplikasi
     terhubung, berkedip cepat = IMU bermasalah.

  Kabel (ESP32-C3 Super Mini)
    MPU6050  VCC -> 3V3   GND -> G   SDA -> GPIO6   SCL -> GPIO7
    Servo putar atas-bawah (di G_Base)            oranye -> GPIO3
    Servo putar kanan-kiri (di belakang kamera)   oranye -> GPIO4   (jika tertukar, pilih "Tertukar" di aplikasi)
    Merah kedua servo -> 5V,  coklat kedua servo -> G,  kapasitor 100-470 uF di 5V/G
    Listrik: power bank 5 V >= 2 A -> kabel USB-A ke USB-C -> port USB-C ESP32.  Tanpa resistor.

  Arduino IDE
    - Boards Manager: "esp32" by Espressif Systems, versi 3.x
    - Tools > Board: "ESP32C3 Dev Module"
    - Tools > USB CDC On Boot: "Enabled"
    - Tools > Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"  (aplikasi web ikut di dalam firmware)
    - Tidak perlu library tambahan. File web_files.h (aplikasi web) harus satu folder dengan file ini.

  Sandi bawaan aplikasi: 123456  (ganti di aplikasi, menu Keamanan)
  Lupa sandi: colok USB, buka Serial Monitor 115200, kirim:  PIN 123456
  Perintah teks (USB atau WiFi): kirim HELP.
*/
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <lwip/sockets.h>
#include <errno.h>
#include <stdarg.h>
#include "esp_system.h"
#include "esp_random.h"
#include "web_files.h"

#define FW_VERSION "4.4"

// ------------------------------------------------------------------------------------ pins
static const int PIN_SDA = 6, PIN_SCL = 7;
static const int PIN_SERVO_A = 3;   // normally the pitch servo (in G_Base)
static const int PIN_SERVO_B = 4;   // normally the roll servo (behind the camera)
static const int PIN_LED = 8;       // onboard blue LED, active LOW

// ------------------------------------------------------------------------------------ constants
static const uint32_t LOOP_US = 5000;  // 200 Hz control loop
static const float CAL_DUS = 150.0f;   // auto-calibration servo move (us), about 14 degrees
// If the camera turns less than CAL_MIN_DEG (it hit the frame or a cable on that side, or the servo
// cannot lift the D435 that way), the move is tried the other way, then bigger (CAL_DUS_BIG, ~24 deg,
// inside the 25 deg envelope) on the freer side. v4.2 tried once, one way, and failed.
static const float CAL_MIN_DEG = 8.0f;
static const float CAL_DUS_BIG = 250.0f;
static const float TIME_DUS = 60.0f;   // auto-calibration timing steps (us), about 5 degrees
static const int TIME_STEPS = 4;       // per servo: up, down, up, down
static const int TIME_STEP_MS = 305;   // 305 ms: each step starts 5 ms later in the 20 ms servo frame
static const float DB_SWING = 40.0f;   // dead band test: come back to the centre from 40 us below / above
static const int DB_STEPS = 8;         // per servo: below, centre, above, centre, twice
// Stillness limit while measuring the gyro bias. v4.2 used 0.6 deg/s and failed at once: two MG90S
// holding a D435 hum by themselves (dead band hunting) and the IMU sits on the camera, so the gyro sees
// ~0.5..1.5 deg/s of buzz even when nobody touches it. Buzz averages out of the 2 s mean (error
// < 0.1 deg/s); only real motion matters. Not still: wait and measure again (up to 5x), then fail.
static const float CAL_STILL_SD = 1.8f;
static const uint8_t CAL_RETRIES = 5;
static const char *DEFAULT_PIN = "123456";
// Measured on the real gimbal (MG90S, clone IMU): prediction 0.7..1.1 and dead band compensation 2.5
// made the kanan-kiri servo shake +-1..3 deg while standing still (the IMU then even read it as
// walking). With both off it stays within +-0.01 deg. The simulation preferred them; the gimbal does not.
static const float PRED_DEFAULT = 0.0f;
// The prediction multiplies the body's turn rate by ~60 ms. The gyro also sees the plastic frame and the
// servo gears ringing (tens of Hz); multiplied like that and sent back to the servos it kept them
// shaking. Body motion and steps are below ~3 Hz, so the rate is low-passed for the prediction only.
#ifndef PRED_LP_HZ
#define PRED_LP_HZ 4.0f
#endif
static const float DBCOMP_DEFAULT = 0.0f;  // us (2.5 = a quarter of the MG90S dead band; shook on the real gimbal)
static const float SRVDB_DEFAULT = 10.0f;  // MG90S as sold: "dead band width 10 us"
static const uint16_t PWMHZ_DEFAULT = 100;  // fine for analog servos like MG90S; 50 if one runs hot, up to 333 for digital servos
static const float SMOOTH_DEFAULT = 0.02f;
static const float GAIT_DEFAULT = 1.0f;
// Smoothness: how gently a new target (preset, slider, switching to Stabil) is approached. Only the
// target glides; the stabilisation itself stays as fast as before.
static const float SOFT_DEFAULT = 0.5f;
// Servo speed and acceleration are the user's choice (there is no servo calibration any more). Slow by
// default: the MG90S lives longest and the camera moves without jolts. Above SPEED_WARN the gimbal and
// the app warn that the servo wears out and runs hot faster.
// Chosen in the physics simulation: 60 deg/s keeps big moves (leaning, switching target) slow and calm
// and still meets the small quick jolts of each step (they need far less than 60 deg/s).
// An acceleration limit was tried as well: every value made the camera follow later and shake more
// (360 deg/s^2 cannot follow walking at all), so it is off by default and only an advanced option.
static const float SPEED_DEFAULT = 60.0f;   // deg/s
// The atas-bawah servo holds the whole D435 and its gears slipped (broke twice) past some shaft
// position. That position is a pulse width, so the limits are kept in us from the centre pulse (they do
// not change when usdeg is changed). Set them in Manual mode: move the servo to the furthest safe point
// and send LIMP down / LIMP up (buttons in the app). Until then +-150 us (~13 deg of shaft).
static const float LIMP_US_DEFAULT = 150.0f;
static const float SPEED_WARN = 200.0f;     // deg/s
static const float ACC_DEFAULT = 0.0f;      // deg/s^2, 0 = no acceleration limit
static const char *DEFAULT_NAME = "Gimbal D435";
static const char *DEFAULT_AP_PASS = "gimbald435";
static const int WS_N = 3;    // phones/tablets connected at the same time
static const int HTTP_N = 4;  // files being sent at the same time (a browser opens several)

enum Mode : uint8_t { MODE_STAB = 0, MODE_CENTER = 1, MODE_MANUAL = 2, MODE_OFF = 3 };
static const char *MODE_NAMES[] = { "stab", "center", "manual", "off" };
enum : uint8_t { TO_USB = 1 };  // destinations: bit 0 = USB, bit 1+i = web app client i
// NOTE: no function may be defined above the last struct. The Arduino IDE puts its generated function
// prototypes just before the first function in this file, and a prototype that names a struct only
// compiles if that struct is already defined there.

// ------------------------------------------------------------------------------------ configuration (flash)
struct Config {
  uint32_t magic;
  uint16_t size;
  uint16_t calOK;  // bit0 = auto calibration done (until 4.3), bit1 = level done, bit2 = gyro done, bit3 = 3D model done
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
  // v4.1
  float ffBody;    // 0..1: turn the servo against the measured body motion right away (feed-forward)
  float pred;      // 0..1.5: how much of the servo's delay to predict ahead
  float srvDelay;  // ms: servo dead time as the IMU sees it (50 Hz frame + electronics + IMU filter)
  float srvTau;    // ms: servo lag after the dead time
  float accTauW;   // s: accelerometer time constant while walking
  float dbComp;    // us: servo dead band compensation
  float srvDb;     // us: servo dead band width as in its datasheet (it stops within half of it on either side)
  uint16_t pwmHz;  // servo pulses per second
  uint8_t wifiOn;  // 0 = WiFi off (USB only)
  uint8_t ditherUs;  // us: small pulse wobble that keeps the servo out of its dead band (0 = off); was padding
  char wifiSsid[33];  // hotspot to join (empty = only the gimbal's own WiFi)
  char wifiPass[65];
  char apPass[33];    // the gimbal's own WiFi (8..32)
  // v4.2
  float gait;  // 0..1: use the learned walking pattern to turn before each step (0 = straight-line only)
  // v4.3
  float soft;  // 0..1: smoothness. A new target glides there instead of jumping (0 = jump)
  // v4.4
  float srvAcc;  // deg/s^2: optional servo acceleration limit (0 = off); maxRate is the speed limit
  float limPUsDn, limPUsUp;  // us from the centre pulse: atas-bawah servo position limits (camera down / up)
  float twinP, twinR;      // deg: camera pitch/roll with both servos in the centre and the body upright (3D model)
  uint32_t crc;
};
struct ConfigV43 {  // firmware 4.3 layout, read once and upgraded
  uint32_t magic;
  uint16_t size;
  uint16_t calOK;
  float pitchSet, rollSet;
  float kp, gain, rateFF, deadband, smooth, maxRate, accTau;
  uint8_t dlpf, autoBias, bootGyro, nod;
  int16_t trimP, trimR;
  int8_t dirP, dirR, swapServo, startMode;
  float usDegP, usDegR, limP, limR;
  int8_t axF, axL, axU, sgF, sgL, sgU, telHz, ledOn;
  float lvlP, lvlR;
  float bias[3];
  char name[24];
  char pin[16];
  float ffBody, pred, srvDelay, srvTau, accTauW, dbComp, srvDb;
  uint16_t pwmHz;
  uint8_t wifiOn, pad41;
  char wifiSsid[33], wifiPass[65], apPass[33];
  float gait, soft;
  uint32_t crc;
};
struct ConfigV42 {  // firmware 4.2 layout, read once and upgraded
  uint32_t magic;
  uint16_t size;
  uint16_t calOK;
  float pitchSet, rollSet;
  float kp, gain, rateFF, deadband, smooth, maxRate, accTau;
  uint8_t dlpf, autoBias, bootGyro, nod;
  int16_t trimP, trimR;
  int8_t dirP, dirR, swapServo, startMode;
  float usDegP, usDegR, limP, limR;
  int8_t axF, axL, axU, sgF, sgL, sgU, telHz, ledOn;
  float lvlP, lvlR;
  float bias[3];
  char name[24];
  char pin[16];
  float ffBody, pred, srvDelay, srvTau, accTauW, dbComp, srvDb;
  uint16_t pwmHz;
  uint8_t wifiOn, pad41;
  char wifiSsid[33], wifiPass[65], apPass[33];
  float gait;
  uint32_t crc;
};
struct ConfigV41 {  // firmware 4.1 layout, read once and upgraded
  uint32_t magic;
  uint16_t size;
  uint16_t calOK;
  float pitchSet, rollSet;
  float kp, gain, rateFF, deadband, smooth, maxRate, accTau;
  uint8_t dlpf, autoBias, bootGyro, nod;
  int16_t trimP, trimR;
  int8_t dirP, dirR, swapServo, startMode;
  float usDegP, usDegR, limP, limR;
  int8_t axF, axL, axU, sgF, sgL, sgU, telHz, ledOn;
  float lvlP, lvlR;
  float bias[3];
  char name[24];
  char pin[16];
  float ffBody, pred, srvDelay, srvTau, accTauW, dbComp, srvDb;
  uint16_t pwmHz;
  uint8_t wifiOn, pad41;
  char wifiSsid[33], wifiPass[65], apPass[33];
  uint32_t crc;
};
struct ConfigV40 {  // firmware 4.0 layout, read once and upgraded
  uint32_t magic;
  uint16_t size;
  uint16_t calOK;
  float pitchSet, rollSet;
  float kp, gain, rateFF, deadband, smooth, maxRate, accTau;
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
  char s[144];
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
  float tth, dSum, tauSum;  // auto: servo timing steps
  float dbLo[2][3], dbHi[2][3];  // auto: gravity at the centre, reached from below / from above
  float tb[64];
  uint16_t tt[64];
  uint8_t tn, tstep, tCount;
  uint8_t retry = 0;  // stillness retries (servos holding the camera hum a little)
  float amp = CAL_DUS;                      // move being tried now (us, signed)
  float ampOf[2] = { CAL_DUS, CAL_DUS };    // move that was used for each servo
  float mTry[3] = { 0, 0, 0 }, mMax[2] = { 0, 0 }, mAcc[2] = { 0, 0 };
  uint8_t tryN = 0;
  bool redo = false;
} cal;
struct Sample {
  uint16_t t;
  int16_t sp, p, sr, r, op, orr;
};
static const int CAP_N = 300;  // 3 s at 100 Hz
static const int MDL_N = 48;   // servo model history, 240 ms at 200 Hz
static const int GAIT_K = 12;  // stride harmonics: 12 x ~0.9 Hz reaches ~11 Hz, a heel strike's main part
struct Gait {
  float h1, l1, l2, l2p, thp;  // stride clock from the left-right sway
  float w, ph, amp;            // stride rate (rad/s), phase, sway amplitude (deg)
  float cth[GAIT_K], sth[GAIT_K], gin[GAIT_K];  // servo phase lag and 1/gain at each harmonic
  float a[GAIT_K], b[GAIT_K];  // learned atas-bawah servo correction over one stride (degrees)
  float u;                     // its value now
  uint8_t tick;
  bool on;
};
struct SrvModel {              // what the servo has really done by now, as the IMU will see it
  float hist[MDL_N];
  uint8_t i;
  float m, rate;
};
static const size_t WS_TX = 6144;
struct WsConn {
  WiFiClient c;
  bool used, authed;
  char nonce[33];
  uint8_t rx[700];  // incoming WebSocket bytes
  uint16_t rn;
  char line[256];  // command being received
  uint16_t ln;
  char tx[WS_TX];  // outgoing text, sent as WebSocket frames
  uint16_t th, tt;
  uint8_t fr[1408];  // frame (or the handshake) being sent
  uint16_t frN, frOff;
  uint32_t lastRx, closeAt;
};
struct HttpConn {  // a browser fetching one file of the app
  WiFiClient c;
  bool used;
  char req[1200];
  uint16_t rn;
  char head[300];  // response header being sent
  uint16_t hn, hoff;
  const uint8_t *body;
  uint32_t len, off, t0;
};
struct DbComp {  // servo dead band compensation state
  float pk;
  int8_t sg;
};

// ------------------------------------------------------------------------------------ runtime state
// IMU
uint8_t imuAddr = 0x68, imuWho = 0;
bool imuOK = false;
uint32_t i2cErr = 0, i2cErrLast = 0;
// I2C clock. 400 kHz reads the 14 IMU bytes in ~0.5 ms instead of ~1.8 ms (less delay in every loop).
// If the IMU wires are too long for it and errors pile up, it drops to 100 kHz by itself and remembers.
uint32_t i2cHz = 400000;
void i2cBegin() {
  Wire.begin(PIN_SDA, PIN_SCL, i2cHz);
  Wire.setTimeOut(4);  // ms: a stuck read must not eat the whole 5 ms loop (was 10)
}
// Accelerometer offset and gain per raw axis. Cheap MPU6050 clones can read e.g. +0.28 g on one axis at
// rest: that bends every angle (25 deg reads as ~20 and creeps after each move) and makes |a| look like
// steps, so the walking detector stays on. Measured with ACC in several still poses, fitted offline.
float accOff[3] = { 0, 0, 0 }, accGain[3] = { 1, 1, 1 };
float accAvgRaw[3] = { 0, 0, 0 };  // uncorrected, ~0.25 s average, printed by ACC
int imuFailRun = 0;
float accRaw[3] = { 0, 0, 1 };     // g, raw IMU axes
float gyrUncal[3] = { 0, 0, 0 };   // deg/s, raw IMU axes, no bias removed
float gyrRaw[3] = { 0, 0, 0 };     // deg/s, raw IMU axes, bias removed
float gyrInt[3] = { 0, 0, 0 };     // deg, rotation summed per raw axis since GYRO reset (diagnosis)
// attitude
float thRaw = 0, phRaw = 0;        // estimator (deg), before level offsets
float gEst[3] = { 0, 0, 1 };       // gravity direction in camera axes (F, L, U), unit length
float walkLvl = 0, walkF = 0;      // walking detector: mean bounce of |a|, and 0..1
// |a| at rest, followed slowly. Clone chips (WHO 0x70/0x74...) read e.g. 0.93 g or 1.07 g when still;
// measured against exactly 1 g that offset looked like walking all the time (gait learning on, the
// accelerometer trusted less, never "still"). The bounce is now measured against this level.
float anRest = 1.0f;
float pitchMeas = 0, rollMeas = 0; // after level offsets (deg)
float pitchRate = 0, rollRate = 0; // Euler rates (deg/s)
float stillTime = 0;
bool isStill = false;
uint32_t accCount = 0, accRejCount = 0;
float accRejPct = 0;
// control
Mode mode = MODE_STAB;
float cmdP = 0, cmdR = 0;    // PID integrator (deg of camera rotation)
float ffP = 0, ffR = 0;      // body feed-forward part of the output
float lastTgtP = 0, lastTgtR = 0;
float slewP = 0, slewR = 0;  // after slew limit
float glP = 0, glR = 0, glVP = 0, glVR = 0;  // target as it glides (deg, deg/s)
bool glInit = false;
float velP = 0, velR = 0;    // speed of the motion profile (deg/s)
float rfP = 0, rfR = 0;      // body turn rate for the prediction, low-passed (deg/s)
float lagP = 0, lagR = 0;    // how far the motion profile is behind the controller's output (deg)
float outP = 0, outR = 0;    // after smoothing
float manP = 0, manR = 0;    // manual mode targets
float usP = 1500, usR = 1500;
bool satP = false, satR = false;
float nodOffset = 0;
uint32_t nodUntil = 0, softStartUntil = 0;
// housekeeping
uint32_t lastTickUs = 0;  // when the last control step really ran
float imuDt = 0;          // time since the last good IMU sample (a failed read must not lose that rotation)
uint32_t lastLoopUs = 0, loopCount = 0, loopHz = 0, statT = 0, saveAt = 0, telT = 0, rebootAt = 0;
uint32_t cpuSum = 0, cpuPeak = 0, cpuUs = 0, cpuMaxUs = 0;  // us of IMU + estimate + control per loop (average, peak)
uint32_t bootCount = 0, brownCount = 0;
esp_reset_reason_t rstReason;
bool firstBoot = false;

// ------------------------------------------------------------------------------------ small helpers
static inline uint8_t toWs(int i) {  // destination bit of web app client i
  return (uint8_t)(2 << i);
}
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
// v4.3 ran a calmed-down "safe" tuning until CAL auto had measured the servos. 4.4 has no servo
// calibration: the user sets direction, wiring and scale, so the full tuning is always used and the
// motion profile (speed and acceleration limits) keeps it gentle.
bool safeTune() {
  return false;
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
  char name[24], pin[16], ssid[33], wpass[65], apass[33];
  uint8_t won = c.wifiOn;
  memcpy(name, c.name, sizeof(name));
  memcpy(pin, c.pin, sizeof(pin));
  memcpy(ssid, c.wifiSsid, sizeof(ssid));
  memcpy(wpass, c.wifiPass, sizeof(wpass));
  memcpy(apass, c.apPass, sizeof(apass));
  memset(&c, 0, sizeof(c));
  c.magic = CFG_MAGIC;
  c.size = sizeof(Config);
  c.pitchSet = 0.0f;  // like a handheld stabiliser: camera straight ahead and level (v4.2: 35 deg down, belt use)
  c.rollSet = 0.0f;
  c.kp = 0.6f;
  c.gain = 10.0f;
  c.rateFF = 0.8f;
  c.deadband = 0.2f;
  c.smooth = SMOOTH_DEFAULT;
  c.maxRate = SPEED_DEFAULT;
  c.srvAcc = ACC_DEFAULT;
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
  c.limP = 50.0f;  // overall cap; the real safe range is limPUsDn / limPUsUp
  c.limPUsDn = LIMP_US_DEFAULT;
  c.limPUsUp = -LIMP_US_DEFAULT;
  c.twinP = 35.0f;  // the 3D model's own pose (belt use) until CAL twin measures the real one
  c.twinR = 0.0f;  // atas-bawah: +-16 deg, the MG90S under the D435 slips its gears beyond that (4.3: 50)
  c.limR = 55.0f;  // kanan-kiri: a 50 deg lean is still corrected, with a little to spare (v4.2: 25)
  c.axF = 0;
  c.axL = 1;
  c.axU = 2;
  c.sgF = 1;
  c.sgL = 1;
  c.sgU = 1;
  c.telHz = 20;
  c.ledOn = 1;
  c.ffBody = 1.0f;
  c.pred = PRED_DEFAULT;
  c.srvDelay = 15.0f;
  c.srvTau = 25.0f;
  c.accTauW = 3.0f;
  c.dbComp = DBCOMP_DEFAULT;
  c.srvDb = SRVDB_DEFAULT;
  c.pwmHz = PWMHZ_DEFAULT;
  c.gait = GAIT_DEFAULT;
  c.soft = SOFT_DEFAULT;
  c.wifiOn = 1;
  strcpy(c.apPass, DEFAULT_AP_PASS);
  if (keepIdentity) {  // name, PIN and WiFi stay, so "back to defaults" does not cut the connection
    memcpy(c.name, name, sizeof(name));
    memcpy(c.pin, pin, sizeof(pin));
    memcpy(c.wifiSsid, ssid, sizeof(ssid));
    memcpy(c.wifiPass, wpass, sizeof(wpass));
    memcpy(c.apPass, apass, sizeof(apass));
    c.wifiOn = won;
    c.wifiSsid[sizeof(c.wifiSsid) - 1] = c.wifiPass[sizeof(c.wifiPass) - 1] = c.apPass[sizeof(c.apPass) - 1] = 0;
    if (strlen(c.apPass) < 8) strcpy(c.apPass, DEFAULT_AP_PASS);
  } else {
    strncpy(c.name, DEFAULT_NAME, sizeof(c.name) - 1);
    strncpy(c.pin, DEFAULT_PIN, sizeof(c.pin) - 1);
  }
  c.name[sizeof(c.name) - 1] = 0;
  c.pin[sizeof(c.pin) - 1] = 0;
}
const char *cfgMigrated = nullptr;  // firmware version the settings came from, when they were upgraded
bool cfgLoad() {
  Config t;
  size_t len = prefs.getBytesLength("cfg");
  if (len == sizeof(Config)) {
    prefs.getBytes("cfg", &t, sizeof(t));
    if (t.magic != CFG_MAGIC || t.size != sizeof(Config)) return false;
    if (t.crc != crc32((const uint8_t *)&t, offsetof(Config, crc))) return false;
  } else if (len == sizeof(ConfigV40)) {  // settings from firmware 4.0: keep them, new settings get defaults
    ConfigV40 o;
    prefs.getBytes("cfg", &o, sizeof(o));
    if (o.magic != CFG_MAGIC || o.size != sizeof(ConfigV40)) return false;
    if (o.crc != crc32((const uint8_t *)&o, offsetof(ConfigV40, crc))) return false;
    memset(&t, 0, sizeof(t));
    cfgDefaults(t, false);
    memcpy(&t, &o, offsetof(ConfigV40, crc));  // same layout up to the old crc
    t.size = sizeof(Config);
    cfgMigrated = "4.0";
  } else if (len == sizeof(ConfigV41)) {  // firmware 4.1: same layout up to its crc
    ConfigV41 o;
    prefs.getBytes("cfg", &o, sizeof(o));
    if (o.magic != CFG_MAGIC || o.size != sizeof(ConfigV41)) return false;
    if (o.crc != crc32((const uint8_t *)&o, offsetof(ConfigV41, crc))) return false;
    memset(&t, 0, sizeof(t));
    cfgDefaults(t, false);
    memcpy(&t, &o, offsetof(ConfigV41, crc));
    t.size = sizeof(Config);
    cfgMigrated = "4.1";
  } else if (len == sizeof(ConfigV42)) {  // firmware 4.2: same layout up to its crc
    ConfigV42 o;
    prefs.getBytes("cfg", &o, sizeof(o));
    if (o.magic != CFG_MAGIC || o.size != sizeof(ConfigV42)) return false;
    if (o.crc != crc32((const uint8_t *)&o, offsetof(ConfigV42, crc))) return false;
    memset(&t, 0, sizeof(t));
    cfgDefaults(t, false);
    memcpy(&t, &o, offsetof(ConfigV42, crc));
    t.size = sizeof(Config);
    t.soft = SOFT_DEFAULT;
    // 4.3 is a handheld stabiliser by default: straight ahead, wide limits (4.2: 35 deg down, +-25)
    t.pitchSet = 0.0f;
    t.limP = fmaxf(t.limP, 50.0f);
    t.limR = fmaxf(t.limR, 55.0f);
    cfgMigrated = "4.2";
  } else if (len == sizeof(ConfigV43)) {  // firmware 4.3: same layout up to its crc
    ConfigV43 o;
    prefs.getBytes("cfg", &o, sizeof(o));
    if (o.magic != CFG_MAGIC || o.size != sizeof(ConfigV43)) return false;
    if (o.crc != crc32((const uint8_t *)&o, offsetof(ConfigV43, crc))) return false;
    memset(&t, 0, sizeof(t));
    cfgDefaults(t, false);
    memcpy(&t, &o, offsetof(ConfigV43, crc));
    t.size = sizeof(Config);
    cfgMigrated = "4.3";
  } else {
    return false;
  }
  // 4.4: slow, gentle servos by default (older firmware ran them at up to 400 deg/s)
  if (cfgMigrated) {
    t.maxRate = SPEED_DEFAULT;
    t.srvAcc = ACC_DEFAULT;
  }
  t.name[sizeof(t.name) - 1] = 0;
  t.pin[sizeof(t.pin) - 1] = 0;
  if (t.axF < 0 || t.axF > 2 || t.axL < 0 || t.axL > 2 || t.axU < 0 || t.axU > 2) return false;
  if (t.startMode > MODE_OFF) t.startMode = MODE_STAB;
  t.ffBody = clampf(t.ffBody, 0.0f, 1.0f);
  t.pred = clampf(t.pred, 0.0f, 1.5f);
  t.srvDelay = clampf(t.srvDelay, 0.0f, 60.0f);
  t.srvTau = clampf(t.srvTau, 5.0f, 120.0f);
  t.accTauW = clampf(t.accTauW, 0.3f, 10.0f);
  t.dbComp = clampf(t.dbComp, 0.0f, 10.0f);
  t.srvDb = clampf(t.srvDb, 0.0f, 40.0f);
  t.gait = clampf(t.gait, 0.0f, 1.0f);
  t.soft = clampf(t.soft, 0.0f, 1.0f);
  t.maxRate = clampf(t.maxRate, 20.0f, 600.0f);
  t.limPUsDn = clampf(t.limPUsDn, -1000.0f, 1000.0f);
  t.limPUsUp = clampf(t.limPUsUp, -1000.0f, 1000.0f);
  if (!(t.twinP > -90.0f && t.twinP < 90.0f)) t.twinP = 35.0f;
  if (!(t.twinR > -90.0f && t.twinR < 90.0f)) t.twinR = 0.0f;
  t.srvAcc = t.srvAcc <= 0.0f ? 0.0f : clampf(t.srvAcc, 200.0f, 20000.0f);
  if (t.pwmHz < 50 || t.pwmHz > 333) t.pwmHz = PWMHZ_DEFAULT;
  t.wifiSsid[sizeof(t.wifiSsid) - 1] = t.wifiPass[sizeof(t.wifiPass) - 1] = t.apPass[sizeof(t.apPass) - 1] = 0;
  if (strlen(t.apPass) < 8) strcpy(t.apPass, DEFAULT_AP_PASS);
  t.name[sizeof(t.name) - 1] = t.pin[sizeof(t.pin) - 1] = 0;
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

// ------------------------------------------------------------------------------------ output (USB + WiFi app)
// Each web app client has its own outgoing text buffer. netPoll() sends from it without ever waiting,
// so a slow phone can never stall the 200 Hz loop.
WsConn ws[WS_N];
uint8_t authFails = 0;
uint32_t lockUntil = 0;

size_t wsUsed(const WsConn &w) {
  return (size_t)((w.th + WS_TX - w.tt) % WS_TX);
}
size_t wsFree(const WsConn &w) {
  return WS_TX - 1 - wsUsed(w);
}
void wsPush(WsConn &w, const char *s, size_t n) {
  for (size_t i = 0; i < n; i++) {
    w.tx[w.th] = s[i];
    w.th = (w.th + 1) % WS_TX;
  }
}
int wsCount(bool authedOnly) {
  int n = 0;
  for (int i = 0; i < WS_N; i++)
    if (ws[i].used && (ws[i].authed || !authedOnly)) n++;
  return n;
}
uint8_t allDest() {
  uint8_t d = TO_USB;
  for (int i = 0; i < WS_N; i++)
    if (ws[i].used && ws[i].authed) d |= toWs(i);
  return d;
}
bool destBusy(uint8_t dest, size_t need) {  // some web client in dest has less than 'need' bytes free
  for (int i = 0; i < WS_N; i++)
    if ((dest & toWs(i)) && ws[i].used && wsFree(ws[i]) < need) return true;
  return false;
}
void usbWrite(const char *s, size_t n) {
  if (!Serial) return;                                  // no computer listening
  if (Serial.availableForWrite() < (int)n + 1) return;  // never block the control loop
  Serial.write((const uint8_t *)s, n);
  Serial.write((uint8_t)'\n');
}
// returns false if a web client had to be skipped (its buffer is full)
bool sendTo(uint8_t dest, const char *s) {
  size_t n = strlen(s);
  bool ok = true;
  if (dest & TO_USB) usbWrite(s, n);
  for (int i = 0; i < WS_N; i++) {
    if (!(dest & toWs(i)) || !ws[i].used) continue;
    if (wsFree(ws[i]) >= n + 1) {
      wsPush(ws[i], s, n);
      wsPush(ws[i], "\n", 1);
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

// ------------------------------------------------------------------------------------ servos (LEDC)
// A servo only takes a new position when a pulse arrives: at 50 Hz it moves in 20 ms stairs, which
// shakes the camera (jello in the RGB video). More pulses per second = smaller stairs. The pulse
// width (500..2500 us) is the same at any rate.
uint16_t pwmHzNow = 50;
void pwmAttach(int pin) {
  pwmHzNow = cfg.pwmHz;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, pwmHzNow, 14);
#else
  ledcSetup(pin == PIN_SERVO_A ? 0 : 1, pwmHzNow, 14);
  ledcAttachPin(pin, pin == PIN_SERVO_A ? 0 : 1);
#endif
}
void pwmSetHz(uint16_t hz) {
  if (hz == pwmHzNow) return;
  pwmHzNow = hz;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcChangeFrequency(PIN_SERVO_A, hz, 14);
  ledcChangeFrequency(PIN_SERVO_B, hz, 14);
#else
  ledcChangeFrequency(0, hz, 14);
  ledcChangeFrequency(1, hz, 14);
#endif
}
void pwmWriteUs(int pin, float us) {  // us <= 0: no pulses (servo limp)
  uint32_t duty = 0;
  if (us > 0) {
    us = clampf(us, 500.0f, 2500.0f);
    duty = (uint32_t)(us * 16383.0f * pwmHzNow / 1e6f + 0.5f);
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
  ok &= mpuWrite(0x1B, 0x10);         // gyro +-1000 deg/s: a hard jolt on stairs must not clip (that bends the attitude)
  ok &= mpuWrite(0x1C, 0x10);         // accel +-8 g: heel strikes while jogging stay unclipped
  // 1 kHz sample rate (was 200 Hz). The 200 Hz loop and a 200 Hz sensor drift against each other: some
  // loops read the same sample twice, others skip one, and the data was up to 5 ms old. At 1 kHz every
  // loop gets a sample at most 1 ms old. The DLPF still sets the bandwidth, so the noise is the same.
  ok &= mpuWrite(0x19, 0);
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
    float ar = a / 4096.0f;
    accAvgRaw[k] += (ar - accAvgRaw[k]) * 0.02f;
    accRaw[k] = (ar - accOff[k]) * accGain[k];
    gyrUncal[k] = g / 32.8f;
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
void gravAngles(float &th, float &ph) {
  th = atan2f(-gEst[0], sqrtf(gEst[1] * gEst[1] + gEst[2] * gEst[2])) * RAD_TO_DEG;
  ph = atan2f(gEst[1], gEst[2]) * RAD_TO_DEG;
}
void attitudeReset() {
  float f, l, u;
  toCam(accRaw, f, l, u);
  float n = sqrtf(f * f + l * l + u * u);
  if (n < 0.3f) {
    f = 0;
    l = 0;
    u = n = 1;
  }
  gEst[0] = f / n;
  gEst[1] = l / n;
  gEst[2] = u / n;
  if (n > 0.5f && n < 1.5f) anRest = n;
  gravAngles(thRaw, phRaw);
  pitchMeas = thRaw - cfg.lvlP;
  rollMeas = phRaw - cfg.lvlR;
}
// Attitude = direction of gravity in camera axes. The gyro turns that direction every 5 ms; the
// accelerometer pulls it back slowly. Done on the vector (not on the angles), so the forward/back
// pushes of each step, which come together with the up/down bounce, average out instead of leaving
// a tilt behind. While walking the accelerometer is trusted less (accTauW) but evenly.
void estimate(float dt) {
  float aF, aL, aU, gF, gL, gU;
  toCam(accRaw, aF, aL, aU);
  toCam(gyrRaw, gF, gL, gU);
  // a direction fixed in the world, seen from the turning camera: dg/dt = -w x g
  float wF = gF * DEG_TO_RAD, wL = gL * DEG_TO_RAD, wU = gU * DEG_TO_RAD;
  float g0 = gEst[0], g1 = gEst[1], g2 = gEst[2];
  gEst[0] = g0 - (wL * g2 - wU * g1) * dt;
  gEst[1] = g1 - (wU * g0 - wF * g2) * dt;
  gEst[2] = g2 - (wF * g1 - wL * g0) * dt;
  // walking detector: the up/down bounce of the steps
  float an = sqrtf(aF * aF + aL * aL + aU * aU);
  if (an > 0.5f && an < 1.5f) anRest += (an - anRest) * dt / (8.0f + dt);  // ~8 s: steps average out
  float dev = fabsf(an - anRest);
  walkLvl += (dev - walkLvl) * dt / (0.5f + dt);
  walkF = clampf((walkLvl - 0.02f) / 0.05f, 0.0f, 1.0f);
  // accelerometer correction: still -> skip shakes (|a| far from 1 g). Walking -> take EVERY sample with
  // the same weight: over whole steps the body's accelerations average to zero, but dropping or
  // down-weighting some of them (heel strikes come at the same point of every step) leaves the forward
  // push of the rest behind as a tilt.
  float w = (1.0f - walkF) * clampf(1.0f - dev / 0.15f, 0.0f, 1.0f) + walkF;
  accCount++;
  if (w < 0.5f) accRejCount++;
  if (an > 0.3f && w > 0.0f) {
    float tau = cfg.accTau + (cfg.accTauW - cfg.accTau) * walkF;
    float k = w * dt / (tau + dt);
    gEst[0] += (aF - gEst[0]) * k;
    gEst[1] += (aL - gEst[1]) * k;
    gEst[2] += (aU - gEst[2]) * k;
  }
  float gn = sqrtf(gEst[0] * gEst[0] + gEst[1] * gEst[1] + gEst[2] * gEst[2]);
  if (gn > 1e-3f) {
    gEst[0] /= gn;
    gEst[1] /= gn;
    gEst[2] /= gn;
  } else {
    attitudeReset();
  }
  gravAngles(thRaw, phRaw);
  // Euler rates (deg/s) for the D term and the body estimate
  float ph = phRaw * DEG_TO_RAD, th = thRaw * DEG_TO_RAD;
  float c = cosf(th);
  if (fabsf(c) < 0.2f) c = (c < 0) ? -0.2f : 0.2f;
  float tanT = sinf(th) / c;
  pitchRate = gL * cosf(ph) - gU * sinf(ph);
  rollRate = gF + tanT * (gL * sinf(ph) + gU * cosf(ph));
  pitchMeas = thRaw - cfg.lvlP;
  rollMeas = phRaw - cfg.lvlR;
  float gm = sqrtf(gF * gF + gL * gL + gU * gU);
  // stillness (used by calibration and slow gyro-bias tracking)
  if (gm < 1.5f && dev < 0.05f) stillTime += dt;
  else stillTime = 0;
  isStill = stillTime > 1.0f;
}

// ------------------------------------------------------------------------------------ servo model
// The servo only reads a new pulse every 20 ms, then needs time to get there, and the IMU filter adds a
// few ms before the camera's motion shows up. The model replays our own output through that delay and
// lag, so "camera angle minus model" is how far the BODY is tilted, without waiting for the PID.
SrvModel mdlP, mdlR;
DbComp dbcP, dbcR;

// ------------------------------------------------------------------------------------ walking pattern
// At every step the body tips forward and back within a few hundredths of a second (the heel strike).
// The servo needs about 40 ms to show a change, so it can only meet that jolt if it starts before it.
// The jolt comes back at the same point of every stride (one left and one right step, which may differ,
// e.g. on stairs), so the camera's remaining atas-bawah error is learned as a sum of stride harmonics on
// a stride clock taken from the left-right sway, and the servo gets the opposite of it ahead of time
// (adaptive feed-forward cancellation). Each harmonic is learned through the servo's own delay and lag
// (measured by CAL auto), so the correction arrives on time. Kanan-kiri does not need it: the sway is
// smooth and the straight-line prediction already follows it. Only while walking with a clear sway; it
// fades out when the walking stops and is kept for the next walk.
Gait gait;
static float wrapPi(float a) {
  while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}
void gaitReset() {
  memset(&gait, 0, sizeof(gait));
}
// how much later and how much smaller each harmonic shows up on the camera (servo delay, lag, smoothing)
static void gaitServoPhase(Gait &g) {
  float d = cfg.srvDelay * 1e-3f, tau = cfg.srvTau * 1e-3f, sm = cfg.smooth;
  for (int j = 0; j < GAIT_K; j++) {
    float w = (j + 1) * g.w;
    float th = w * d + atanf(w * tau) + atanf(w * sm);
    g.cth[j] = cosf(th);
    g.sth[j] = sinf(th);
    g.gin[j] = fminf(sqrtf((1.0f + w * w * tau * tau) * (1.0f + w * w * sm * sm)), 3.0f);
  }
}
// bodyRoll: left-right body sway (deg); err: atas-bawah camera error (target - measured)
void gaitStep(float bodyRoll, float err, bool learn, float dt) {
  Gait &g = gait;
  const float TWO_PI_F = 2.0f * (float)M_PI;
  if (!g.on) {
    gaitReset();
    g.on = true;
    g.w = TWO_PI_F * 0.9f;
    g.h1 = bodyRoll;
    gaitServoPhase(g);
  }
  // stride clock: phase of the left-right sway, band-passed to 0.4..1.3 Hz (phase portrait). The clock
  // follows that phase within ~1/3 s; its small wobble within each stride repeats every stride and so
  // is simply part of the learned pattern.
  g.h1 += (bodyRoll - g.h1) * dt / 0.4f;
  float k = TWO_PI_F * 1.3f * dt;
  g.l1 += (bodyRoll - g.h1 - g.l1) * k;
  g.l2 += (g.l1 - g.l2) * k;
  float d = (g.l2 - g.l2p) / dt;
  g.l2p = g.l2;
  float q = d / g.w;
  float th = atan2f(g.l2, q);
  float dth = wrapPi(th - g.thp);
  g.thp = th;
  g.amp += (sqrtf(g.l2 * g.l2 + q * q) - g.amp) * dt / 0.5f;
  g.w += (clampf(dth / dt, TWO_PI_F * 0.4f, TWO_PI_F * 1.8f) - g.w) * dt / 1.0f;
  g.ph = wrapPi(g.ph + g.w * dt + 3.0f * dt * wrapPi(th - g.ph));
  if (++g.tick >= 10) {  // every 50 ms is enough: the stride rate changes slowly
    g.tick = 0;
    gaitServoPhase(g);
  }
  // harmonics of the phase (angle-addition recurrence: one sin/cos per step)
  float sk[GAIT_K], ck[GAIT_K];
  float s1 = sinf(g.ph), c1 = cosf(g.ph);
  sk[0] = s1, ck[0] = c1;
  for (int j = 1; j < GAIT_K; j++) {
    sk[j] = sk[j - 1] * c1 + ck[j - 1] * s1;
    ck[j] = ck[j - 1] * c1 - sk[j - 1] * s1;
  }
  float u = 0;
  for (int j = 0; j < GAIT_K; j++) u += g.a[j] * sk[j] + g.b[j] * ck[j];
  g.u = u;
  if (!learn || g.amp < 0.5f) return;  // no clear left-right sway: the clock cannot be trusted
  const float gam = 0.005f, leak = 1.0f - 0.03f * dt;  // learns in ~2 s, forgets an old pattern in ~30 s
  for (int j = 0; j < GAIT_K; j++) {
    // the harmonic as the camera will see it: delayed by the servo's phase lag
    float sd = sk[j] * g.cth[j] - ck[j] * g.sth[j], cd = ck[j] * g.cth[j] + sk[j] * g.sth[j];
    float e = gam * g.gin[j] * err;
    g.a[j] = clampf(g.a[j] * leak + e * sd, -6.0f, 6.0f);
    g.b[j] = clampf(g.b[j] * leak + e * cd, -6.0f, 6.0f);
  }
}
float gaitAdd() {
  return cfg.gait * walkF * fminf(gait.amp / 0.8f, 1.0f) * gait.u;
}
void srvModelReset(SrvModel &s, float v) {
  for (int k = 0; k < MDL_N; k++) s.hist[k] = v;
  s.i = 0;
  s.m = v;
  s.rate = 0;
}
void srvModelStep(SrvModel &s, float out, float usDeg, float dt) {
  s.hist[s.i] = out;
  s.i = (s.i + 1) % MDL_N;
  float d = clampf(cfg.srvDelay * 1e-3f / fmaxf(dt, 1e-4f), 0.0f, (float)(MDL_N - 2));
  int d0 = (int)d;
  float fr = d - d0;
  float a = s.hist[(s.i + MDL_N - 1 - d0) % MDL_N], b = s.hist[(s.i + MDL_N - 2 - d0) % MDL_N];
  float delayed = a + (b - a) * fr;
  float k = 1.0f - expf(-dt / fmaxf(cfg.srvTau * 1e-3f, 0.002f));
  // like the servo, the model stays put for small changes and stops at the edge of the dead band
  // (minus what dbComp already makes up for)
  float db = fmaxf(0.5f * cfg.srvDb - cfg.dbComp, 0.0f) / fmaxf(usDeg, 1.0f);
  float err = delayed - s.m, prev = s.m;
  if (fabsf(err) > db) s.m += (err - copysignf(db, err)) * k;
  s.rate = (s.m - prev) / dt;
}
void controlReset() {
  cmdP = cmdR = slewP = slewR = outP = outR = ffP = ffR = lastTgtP = lastTgtR = 0;
  velP = velR = lagP = lagR = 0;
  rfP = rfR = 0;
  glInit = false;
  srvModelReset(mdlP, 0);
  srvModelReset(mdlR, 0);
  dbcP = DbComp();
  dbcR = DbComp();
}
// A servo stops as soon as it is inside its dead band, so it ends up short of the command in the
// direction it was moving. Adding the dead band in that direction lets it stop on the command.
// The direction only flips after a real reversal, so noise cannot make it rattle (together with the
// pulse hold in control(), which keeps sensor noise from reaching the pulse at all).
float dbCompApply(DbComp &c, float us) {
  if (cfg.dbComp <= 0 || mode != MODE_STAB || safeTune()) {  // centre / manual / off / not calibrated: exact pulse
    c.pk = us;
    c.sg = 0;
    return us;
  }
  const float h = fmaxf(1.0f, 0.3f * cfg.dbComp);  // a big backlash jump only after a real reversal, not on noise
  if (c.sg >= 0 && us < c.pk - h) {
    c.sg = -1;
    c.pk = us;
  } else if (c.sg <= 0 && us > c.pk + h) {
    c.sg = 1;
    c.pk = us;
  } else if ((c.sg > 0 && us > c.pk) || (c.sg < 0 && us < c.pk)) {
    c.pk = us;
  }
  return us + c.sg * cfg.dbComp;
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
// microseconds per degree of each servo. 3) four quick 60 us steps per servo: how long until the camera
// starts to turn (delay) and how fast it gets there (lag), for the servo model. 4) each servo comes back
// to the centre from below and from above, twice: it stops short of the centre on the side it came from,
// so the two resting angles differ by its dead band (plus any gear play). Every servo sold as "MG90S" is
// a little different, so this is measured rather than taken from the listing.
enum { AP_SETTLE = 0, AP_GYRO, AP_BEFORE, AP_OUT, AP_HOLD, AP_BACK, AP_TIME, AP_DB };
void calDbStart(uint8_t step) {
  calPhase(AP_DB);
  cal.tstep = step;
  float *mv = (cal.servo == 0) ? &cal.usA : &cal.usB;
  *mv = (step % 2) ? 0.0f : ((step % 4 == 0) ? -DB_SWING : DB_SWING);
}
void calTimeStart(uint8_t step) {
  calPhase(AP_TIME);
  cal.tstep = step;
  cal.tn = 0;
  cal.tth = 0;
  float *mv = (cal.servo == 0) ? &cal.usA : &cal.usB;
  *mv = (step % 2 == 0) ? copysignf(TIME_DUS, cal.ampOf[cal.servo]) : 0;  // towards the free side
}
// One step recorded as camera angle about that servo's axis vs time. A servo behaves like a dead time
// followed by a lag: t10 = delay + 0.105 lag, t63 = delay + lag.
void calTimeAnalyse() {
  int n = cal.tn;
  if (n < 40) return;
  float fin = 0;
  int c = 0;
  for (int i = 0; i < n; i++) {
    if (cal.tt[i] >= 200 && cal.tt[i] < 300) {
      fin += cal.tb[i];
      c++;
    }
  }
  if (!c) return;
  fin /= c;
  if (fabsf(fin) < 1.0f) return;
  float t10 = -1, t63 = -1, pv = 0, pt = 0;
  for (int i = 0; i < n; i++) {
    float v = cal.tb[i] / fin, t = cal.tt[i];
    if (t10 < 0 && v >= 0.1f) t10 = pt + (t - pt) * (0.1f - pv) / fmaxf(v - pv, 1e-3f);
    if (v >= 0.632f) {
      t63 = pt + (t - pt) * (0.632f - pv) / fmaxf(v - pv, 1e-3f);
      break;
    }
    pv = v;
    pt = t;
  }
  if (t10 < 0 || t63 <= t10) return;
  float tau = (t63 - t10) / 0.8946f, d = t10 - 0.1054f * tau;
  if (tau < 3.0f || tau > 150.0f || d < 0.0f || d > 80.0f) return;
  cal.dSum += d;
  cal.tauSum += tau;
  cal.tCount++;
}
void calAutoSolve() {
  float m[2], n[2][3], c[2];
  for (int s = 0; s < 2; s++) {
    m[s] = vnorm(cal.rot[s]);
    if (m[s] < 3.0f) {
      static char msg[200];
      snprintf(msg, sizeof(msg),
               "Servo di GPIO%d hampir tidak bergerak (paling jauh %.1f derajat dari 3 percobaan: dua arah dan gerak besar). "
               "Periksa kabel sinyal, 5V, G, sekrup horn, dan apakah dudukan tertahan rangka/kabel.",
               s == 0 ? 3 : 4, cal.mMax[s]);
      calFail(msg);
      return;
    }
    for (int k = 0; k < 3; k++) n[s][k] = cal.rot[s][k] / m[s];
    c[s] = fabsf(vdot(n[s], cal.up));
  }
  // Roll servo turns the camera about its optical axis. With the camera looking down, that axis is closer
  // to vertical than the pitch servo's, which tells the two apart. With the camera straight ahead (as a
  // handheld stabiliser is calibrated) both axes are level and gravity cannot tell them apart: then the
  // wiring of the schematic is taken (GPIO3 = atas-bawah, unless an earlier calibration found it swapped).
  bool levelCam = fabsf(c[1] - c[0]) < 0.1f;
  int rollS = levelCam ? (cfg.swapServo ? 0 : 1) : ((c[1] >= c[0]) ? 1 : 0);
  int pitchS = 1 - rollS;
  int8_t newSwap = (rollS == 0) ? 1 : 0;
  // F axis = roll servo rotation axis, pointing down-forward (F . up < 0)
  int iF = argmaxAbs(n[rollS], -1);
  if (fabsf(n[rollS][iF]) < 0.8f) {
    calFail("IMU tidak sejajar dengan kamera atau dudukan bergerak saat diukur.");
    return;
  }
  // forward = the end of the optical axis that points down. Camera (nearly) level: keep the forward
  // direction of the previous calibration if it was the same IMU axis, otherwise assume +.
  int8_t sF;
  bool fGuess = fabsf(cal.up[iF]) < 0.12f;
  if (!fGuess) sF = (cal.up[iF] > 0) ? -1 : 1;
  else sF = (cfg.axF == iF) ? cfg.sgF : 1;
  // U axis = what is left of 'up' after removing F
  float w[3] = { cal.up[0], cal.up[1], cal.up[2] };
  w[iF] = 0;
  int iU = argmaxAbs(w, iF);
  if (fabsf(w[iU]) < 0.5f) {  // optical axis nearly vertical: "up" of the picture cannot be told from gravity
    static char msg[180];
    snprintf(msg, sizeof(msg),
             "Kamera menghadap hampir tegak lurus ke %s (%.0f derajat dari datar). Posisikan gimbal agar kamera lebih mendatar "
             "(lurus sampai menunduk 60 derajat), lalu ulangi.",
             (cal.up[iF] * sF > 0) ? "atas" : "bawah", asinf(clampf(fabsf(cal.up[iF]), 0.0f, 1.0f)) * RAD_TO_DEG);
    calFail(msg);
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
  // A servo stops at the edge of its dead band, on the side it came from. The +150 us position and the
  // centre reached from below were both approached from below, so they are exactly 150 us apart.
  float th0, ph0, th1, ph1, th2, ph2, th3, ph3, thH, phH, thL, phL;
  // (a move downwards was approached from above, so it pairs with the centre reached from above)
  float aP = cal.ampOf[pitchS], aR = cal.ampOf[rollS];
  accAngles(aP > 0 ? cal.dbLo[pitchS] : cal.dbHi[pitchS], th0, ph0);
  accAngles(cal.aAfter[pitchS], th1, ph1);
  accAngles(aR > 0 ? cal.dbLo[rollS] : cal.dbHi[rollS], th2, ph2);
  accAngles(cal.aAfter[rollS], th3, ph3);
  float dTh = th1 - th0, dPh = wrap180(ph3 - ph2);
  float usDegP = fabsf(aP) / fmaxf(fabsf(dTh), 0.01f), usDegR = fabsf(aR) / fmaxf(fabsf(dPh), 0.01f);
  // the centre reached from above minus from below = the dead band
  accAngles(cal.dbHi[pitchS], thH, phH);
  accAngles(cal.dbLo[pitchS], thL, phL);
  float dbP = fabsf(thH - thL) * usDegP;
  accAngles(cal.dbHi[rollS], thH, phH);
  accAngles(cal.dbLo[rollS], thL, phL);
  float dbR = fabsf(wrap180(phH - phL)) * usDegR;
  // 5..20: a loaded MG90S with gear play turns less than the 14 deg it would without the D435 (v4.2: 6..16)
  // 5..60: a servo that is stiff or weak under the D435 (a tight pivot, a pulling cable) still turns in
  // proportion, just less per us. Its real scale is used, so the loop gain stays right (v4.2: 6..16).
  if (usDegP < 5.0f || usDegP > 60.0f || usDegR < 5.0f || usDegR > 60.0f) {
    cfg.axF = oF;
    cfg.axL = oL;
    cfg.axU = oU;
    cfg.sgF = osF;
    cfg.sgL = osL;
    cfg.sgU = osU;
    static char msg[220];
    snprintf(msg, sizeof(msg),
             "Gerak servo tidak wajar: atas-bawah %.1f (gyro %.1f), kanan-kiri %.1f (gyro %.1f) derajat untuk %.0f/%.0f us. "
             "Servo tertahan, horn slip, atau beban kamera tidak seimbang di porosnya?",
             dTh, m[pitchS], dPh, m[rollS], fabsf(aP), fabsf(aR));
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
  cfg.dirP = ((dTh > 0) == (aP > 0)) ? 1 : -1;
  cfg.dirR = ((dPh > 0) == (aR > 0)) ? 1 : -1;
  cfg.usDegP = usDegP;
  cfg.usDegR = usDegR;
  bool dbOk = dbP < 40.0f && dbR < 40.0f;
  if (dbOk) {
    cfg.srvDb = clampf(0.5f * (dbP + dbR), 1.0f, 40.0f);
    cfg.dbComp = 0.25f * cfg.srvDb;  // half of what the servo stops short: never more than it really does
  }
  bool timed = cal.tCount >= TIME_STEPS;  // at least half of the 8 steps readable
  if (timed) {
    cfg.srvDelay = clampf(cal.dSum / cal.tCount, 0.0f, 60.0f);
    cfg.srvTau = clampf(cal.tauSum / cal.tCount, 5.0f, 120.0f);
  }
  cfg.calOK |= 1 | 4;
  if (mode == MODE_CENTER && cfg.startMode == MODE_STAB) mode = MODE_STAB;  // calibrated: start stabilising
  if (mapChanged) {
    cfg.lvlP = 0;
    cfg.lvlR = 0;
    cfg.calOK &= ~2;
  }
  cfgSave();
  attitudeReset();
  controlReset();
  char ax[8];
  axesString(ax, cfg.axF, cfg.sgF, cfg.axL, cfg.sgL, cfg.axU, cfg.sgU);
  sendf(allDest() | cal.src,
        "CAL DONE auto {\"axes\":\"%s\",\"swap\":%d,\"dirp\":%d,\"dirr\":%d,\"usdegp\":%.2f,\"usdegr\":%.2f,"
        "\"gyrop\":%.1f,\"gyror\":%.1f,\"accp\":%.1f,\"accr\":%.1f,\"align\":%.2f,\"bias\":[%.2f,%.2f,%.2f],\"levelreset\":%d,"
        "\"sdelay\":%.1f,\"stau\":%.1f,\"tsteps\":%d,\"sdbp\":%.1f,\"sdbr\":%.1f,\"sdb\":%.1f,\"dbcomp\":%.1f}",
        ax, cfg.swapServo, cfg.dirP, cfg.dirR, cfg.usDegP, cfg.usDegR, m[pitchS], m[rollS], dTh, dPh, pitchAlign,
        cfg.bias[0], cfg.bias[1], cfg.bias[2], mapChanged ? 1 : 0, cfg.srvDelay, cfg.srvTau, cal.tCount, dbP, dbR, cfg.srvDb,
        cfg.dbComp);
  event("Kalibrasi otomatis selesai: sumbu %s, kabel servo %s, %.1f/%.1f us per derajat", ax, cfg.swapServo ? "tertukar (sudah dikompensasi)" : "benar",
        cfg.usDegP, cfg.usDegR);
  if (levelCam)
    event("Kamera lurus saat kalibrasi: servo atas-bawah dianggap di GPIO%d sesuai kabel. Cek di mode Manual: slider atas-bawah harus "
          "membuat kamera mengangguk. Kalau malah memiringkan kanan-kiri, kirim SET swap %d lalu ulangi kalibrasi (atau kalibrasi dengan kamera menunduk).",
          cfg.swapServo ? 4 : 3, cfg.swapServo ? 0 : 1);
  if (fGuess) event("Arah depan kamera diambil dari kalibrasi sebelumnya (kamera lurus saat diukur).");
  if (timed) event("Kecepatan servo: jeda %.0f ms, waktu tanggap %.0f ms", cfg.srvDelay, cfg.srvTau);
  else event("Kecepatan servo tidak terukur jelas (%d/8 langkah); memakai %.0f/%.0f ms", cal.tCount, cfg.srvDelay, cfg.srvTau);
  if (cfg.usDegP > 20.0f || cfg.usDegR > 20.0f)
    event("Servo %s bergerak lebih sedikit dari normal (%.0f us per derajat, normal sekitar 11): engsel seret, kabel kamera menahan, "
          "atau daya kurang. Hasil ini dipakai, tetapi jangkauan koreksinya lebih kecil.",
          cfg.usDegP > cfg.usDegR ? "atas-bawah" : "kanan-kiri", fmaxf(cfg.usDegP, cfg.usDegR));
  if (dbOk) event("Zona mati servo: atas-bawah %.0f us, kanan-kiri %.0f us (dipakai %.0f us)", dbP, dbR, cfg.srvDb);
  else event("Zona mati servo tidak terukur jelas; memakai %.0f us", cfg.srvDb);
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
      if (sd > CAL_STILL_SD) {
        if (++cal.retry <= CAL_RETRIES) {
          sendf(allDest() | cal.src, "CAL STEP wait %.1f", sd);
          calPhase(0);
          return;
        }
        static char msg[112];
        snprintf(msg, sizeof(msg), "Gimbal bergerak (getar gyro %.1f derajat/s, batas %.1f). Letakkan diam, lalu ulangi.", sd, CAL_STILL_SD);
        calFail(msg);
        return;
      }
      for (int k = 0; k < 3; k++) cfg.bias[k] = cal.s[k] / cal.n;
      cfg.calOK |= 4;
      cfgSave();
      sendf(allDest() | cal.src, "CAL DONE gyro {\"bias\":[%.3f,%.3f,%.3f],\"noise\":%.3f}", cfg.bias[0], cfg.bias[1], cfg.bias[2], sd);
      event("Kalibrasi gyro selesai (derau %.2f derajat/s)", sd);
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
        calFail("Kamera bergerak saat diukur. Tahan diam, lalu ulangi.");
        return;
      }
      if (cal.useP) cfg.lvlP = cal.thSum / cal.n - cal.refP;
      if (cal.useR) cfg.lvlR = cal.phSum / cal.n - cal.refR;
      cfg.calOK |= 2;
      cfgSave();
      attitudeReset();
      sendf(allDest() | cal.src, "CAL DONE level {\"lvlp\":%.2f,\"lvlr\":%.2f}", cfg.lvlP, cfg.lvlR);
      event("Kalibrasi datar selesai: koreksi atas-bawah %.2f, kanan-kiri %.2f derajat", cfg.lvlP, cfg.lvlR);
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
        if (sd > CAL_STILL_SD) {
          if (++cal.retry <= CAL_RETRIES) {  // probably a touch or a bump: measure again
            sendf(allDest() | cal.src, "CAL STEP wait %.1f", sd);
            calPhase(AP_GYRO);
            return;
          }
          static char msg[128];
          snprintf(msg, sizeof(msg), "Gimbal bergerak (getar gyro %.1f derajat/s, batas %.1f). Jepit diam dan tegak seperti saat dipakai, lalu ulangi.", sd,
                   CAL_STILL_SD);
          calFail(msg);
          return;
        }
        for (int k = 0; k < 3; k++) {
          cfg.bias[k] = cal.s[k] / cal.n;
          cal.up[k] = cal.a[k] / cal.n;
        }
        float un = vnorm(cal.up);
        if (un < 0.85f || un > 1.15f) {
          calFail("Pembacaan akselerometer tidak wajar. Periksa IMU.");
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
    case AP_OUT:  // ramp to the move (+150 us first) in 0.6 s, integrate gyro
      *mv = cal.amp * fminf(el / 600.0f, 1.0f);
      for (int k = 0; k < 3; k++) cal.rot[cal.servo][k] += gyrRaw[k] * dt;
      if (el >= 600) calPhase(AP_HOLD);
      break;
    case AP_HOLD:  // keep integrating 0.4 s, then average accel 0.4 s
      *mv = cal.amp;
      if (el < 400) {
        for (int k = 0; k < 3; k++) cal.rot[cal.servo][k] += gyrRaw[k] * dt;
      } else {
        for (int k = 0; k < 3; k++) cal.a[k] += accRaw[k];
        cal.n++;
      }
      if (el >= 800) {
        for (int k = 0; k < 3; k++) cal.aAfter[cal.servo][k] = cal.a[k] / cal.n;
        // How far it really went: the gyro sees the turn, the accelerometer where the camera ended up. A
        // servo that cannot hold the D435 turns and then sags back, so the smaller of the two counts.
        // (The gravity change is smaller than the turn when the axis is not level: divide by sin.)
        float mg = vnorm(cal.rot[cal.servo]), m = mg;
        if (mg > 0.5f) {
          float nb = vnorm(cal.aBefore[cal.servo]), na = vnorm(cal.aAfter[cal.servo]);
          if (nb > 0.5f && na > 0.5f) {
            float cg = clampf(vdot(cal.aBefore[cal.servo], cal.aAfter[cal.servo]) / (nb * na), -1.0f, 1.0f);
            float ax = fabsf(vdot(cal.rot[cal.servo], cal.up)) / mg;
            // only when the servo axis is fairly level: about a vertical axis gravity does not change at all
            if (ax < 0.7f) m = fminf(mg, acosf(cg) * RAD_TO_DEG / sqrtf(1.0f - ax * ax));
          }
        }
        cal.mAcc[cal.servo] = m;
        cal.mTry[cal.tryN] = m;
        cal.mMax[cal.servo] = fmaxf(cal.mMax[cal.servo], m);
        cal.ampOf[cal.servo] = cal.amp;
        if (m < CAL_MIN_DEG && cal.tryN < 2) {  // blocked or too weak this way: try again
          cal.tryN++;
          cal.amp = (cal.tryN == 1) ? -CAL_DUS : (cal.mTry[1] > cal.mTry[0] ? -CAL_DUS_BIG : CAL_DUS_BIG);
          cal.redo = true;
          sendf(allDest() | cal.src, "CAL STEP servo%d move %.1f", cal.servo == 0 ? 3 : 4, m);
        }
        calPhase(AP_BACK);
      }
      break;
    case AP_BACK:  // ramp back in 0.6 s, settle 0.3 s
      *mv = cal.ampOf[cal.servo] * (1.0f - fminf(el / 600.0f, 1.0f));
      if (el >= 900) {
        if (cal.redo) {  // same servo again with the next move
          cal.redo = false;
          calPhase(AP_BEFORE);
        } else if (cal.servo == 0) {
          cal.tryN = 0;
          cal.amp = CAL_DUS;
          cal.servo = 1;
          calPhase(AP_BEFORE);
        } else {
          cal.servo = 0;
          cal.dSum = cal.tauSum = 0;
          cal.tCount = 0;
          sendf(allDest() | cal.src, "CAL STEP timing");
          calTimeStart(0);
        }
      }
      break;
    case AP_TIME: {  // record the camera turning about this servo's axis
      float m = vnorm(cal.rot[cal.servo]);
      if (m > 1.0f && cal.tn < 64 && el < 300) {
        float r = (gyrRaw[0] * cal.rot[cal.servo][0] + gyrRaw[1] * cal.rot[cal.servo][1] + gyrRaw[2] * cal.rot[cal.servo][2]) / m;
        cal.tth += r * dt;
        cal.tb[cal.tn] = cal.tth;
        cal.tt[cal.tn] = el;
        cal.tn++;
      }
      if (el >= (uint32_t)TIME_STEP_MS) {
        calTimeAnalyse();
        if (cal.tstep + 1 < TIME_STEPS) {
          calTimeStart(cal.tstep + 1);
        } else if (cal.servo == 0) {
          cal.servo = 1;
          calTimeStart(0);
        } else {
          cal.servo = 0;
          memset(cal.dbLo, 0, sizeof(cal.dbLo));
          memset(cal.dbHi, 0, sizeof(cal.dbHi));
          sendf(allDest() | cal.src, "CAL STEP deadband");
          calDbStart(0);
        }
      }
      break;
    }
    case AP_DB: {  // swing 250 ms; back at the centre: settle 300 ms, average gravity 300 ms
      bool atCentre = cal.tstep % 2;
      if (atCentre && el >= 300) {
        float(*dst)[3] = (cal.tstep % 4 == 1) ? cal.dbLo : cal.dbHi;
        for (int k = 0; k < 3; k++) dst[cal.servo][k] += accRaw[k];
      }
      if (el >= (atCentre ? 600u : 250u)) {
        if (cal.tstep + 1 < DB_STEPS) {
          calDbStart(cal.tstep + 1);
        } else if (cal.servo == 0) {
          cal.servo = 1;
          calDbStart(0);
        } else {
          calAutoSolve();
        }
      }
      break;
    }
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
  } else if (capState == 2) {  // dump a few lines per loop without flooding the web clients
    uint8_t dest = allDest() | capSrc;
    for (int k = 0; k < 4; k++) {
      if (destBusy(dest, 600)) return;
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
// Stabilise = feed-forward + PID:
//  1) body = camera - servo model. Turn the servo to (target - body), predicted ahead by the servo's
//     delay, so the servo moves at the same moment as the body instead of after the camera has tilted.
//  2) PID on the camera error mops up what the model gets wrong (servo scale, horn, load) and keeps the
//     camera exactly on target. I = gain, P = kp, D = ff (gyro rate).
float pidStep(float &cmd, float e, float rate, float ff, float lo, float hi, float lag, float dt) {
  float kp = cfg.kp, ki = cfg.gain;
  // While the motion profile is still on its way (slow servo setting), the camera error is expected:
  // integrating it would wind up and swing past the target once the servo gets there.
  if ((lag > 1.0f && e > 0) || (lag < -1.0f && e < 0)) ki = 0.0f;
  float i = clampf(cmd + (ki * e - cfg.rateFF * rate) * dt, lo, hi);
  float tot = ff + i + kp * e;
  if (!((tot > hi && e > 0) || (tot < lo && e < 0))) cmd = i;  // no wind-up into the end stop
  return ff + cmd + kp * e;
}
// Servo motion limit. The servo command moves toward the controller's output at no more than maxrate
// ("kecepatan servo", the user's choice). The PID integrator waits while the command is still on its way
// (see pidStep), so a slow setting arrives without swinging past the target. With srvacc > 0 the speed
// also ramps up and down (starts gently, brakes onto the target without overshoot).
void profStep(float &x, float &v, float tgt, float vmax, float amax, float dt) {
  if (amax <= 0.0f) {  // speed limit only (default)
    x += clampf(tgt - x, -vmax * dt, vmax * dt);
    v = 0.0f;
    return;
  }
  float e = tgt - x;
  float ad = amax * dt;
  // fastest speed from which it can still stop on the target (discrete form: no overshoot)
  float vb = sqrtf(2.0f * amax * fabsf(e) + ad * ad) - ad;
  float vd = copysignf(fminf(vb, vmax), e);
  v += clampf(vd - v, -ad, ad);
  float nx = x + v * dt;
  if ((tgt - nx) * e <= 0.0f && fabsf(v) <= 2.0f * ad) {  // arrived
    x = tgt;
    v = 0.0f;
  } else {
    x = nx;
  }
}
void control(float dt) {
  if (cal.kind == CAL_AUTO) {  // calibration drives the two pins directly
    float tA = cfg.swapServo ? cfg.trimR : cfg.trimP;
    float tB = cfg.swapServo ? cfg.trimP : cfg.trimR;
    pwmWriteUs(PIN_SERVO_A, 1500 + tA + cal.usA);
    pwmWriteUs(PIN_SERVO_B, 1500 + tB + cal.usB);
    return;
  }
  if (calFreeze()) return;  // level calibration: hold the outputs exactly where they are
  // a servo can only be driven between 500 and 2500 us: with a large us-per-degree that is fewer degrees
  float limP = fminf(cfg.limP, 950.0f / fmaxf(cfg.usDegP, 1.0f)), limR = fminf(cfg.limR, 950.0f / fmaxf(cfg.usDegR, 1.0f));
  // atas-bawah: the safe shaft range in us, turned into degrees with the current scale and direction.
  // It always includes the centre. Manual mode uses only the overall cap, so the limits can be found.
  float uA = cfg.limPUsDn / (cfg.dirP * fmaxf(cfg.usDegP, 1.0f)), uB = cfg.limPUsUp / (cfg.dirP * fmaxf(cfg.usDegP, 1.0f));
  float loP = fmaxf(-limP, fminf(fminf(uA, uB), 0.0f)), hiP = fminf(limP, fmaxf(fmaxf(uA, uB), 0.0f));
  if (mode == MODE_MANUAL) {
    loP = -limP;
    hiP = limP;
  }
  float tgtP = 0, tgtR = 0;
  bool wasSatP = satP;  // last cycle's saturation (gait learning must not learn against the end stop)
  satP = satR = false;
  if (nodUntil && (int32_t)(millis() - nodUntil) >= 0) {
    nodUntil = 0;
    nodOffset = 0;
  }
  if (mode == MODE_STAB && imuOK && cal.kind == CAL_IDLE) {
    // Target glide: a new target (preset, slider, switching to Stabil) is approached along a smooth,
    // critically damped path from where the camera points, instead of a jump. The step test offset is
    // added after it, so the measured response stays the bare loop.
    float userP = cfg.pitchSet + nodOffset, userR = cfg.rollSet;
    if (!glInit) {
      glP = pitchMeas;
      glR = rollMeas;
      glVP = glVR = 0;
      glInit = true;
    }
    if (cfg.soft > 0.01f) {
      float wn = 1.0f / (0.06f + 0.5f * cfg.soft);  // soft 0.5: ~0.3 s time constant, ~1 s to settle
      glVP += (wn * wn * (userP - glP) - 2.0f * wn * glVP) * dt;
      glVR += (wn * wn * (userR - glR) - 2.0f * wn * glVR) * dt;
      glP += glVP * dt;
      glR += glVR * dt;
    } else {
      glP = userP;
      glR = userR;
      glVP = glVR = 0;
    }
    float spP = glP + stepOffP, spR = glR + stepOffR;
    float eP = softDB(spP - pitchMeas, cfg.deadband);
    float eR = softDB(spR - rollMeas, cfg.deadband);
    bool safe = safeTune();
    float ffb = safe ? 0.0f : cfg.ffBody, pred = safe ? 0.0f : cfg.pred;
    float lead = pred * (cfg.srvDelay + cfg.srvTau + cfg.smooth * 1000.0f) * 1e-3f;
    float bodyP = pitchMeas - mdlP.m, bodyR = rollMeas - mdlR.m;
    float kr = dt / (1.0f / (2.0f * (float)M_PI * PRED_LP_HZ) + dt);
    rfP += (pitchRate - mdlP.rate - rfP) * kr;
    rfR += (rollRate - mdlR.rate - rfR) * kr;
    float bRateP = rfP, bRateR = rfR;
    float lin[2] = {bodyP + lead * bRateP, bodyR + lead * bRateR};
    gaitStep(bodyR, spP - pitchMeas, walkF > 0.5f && !wasSatP && cfg.gait > 0 && !safe, dt);
    ffP = ffb * (spP - lin[0]) + gaitAdd();
    ffR = ffb * (spR - lin[1]);
    float totP = pidStep(cmdP, eP, pitchRate, ffP, loP, hiP, lagP, dt);
    float totR = pidStep(cmdR, eR, rollRate, ffR, -limR, limR, lagR, dt);
    satP = totP >= hiP || totP <= loP;
    satR = totR >= limR || totR <= -limR;
    tgtP = clampf(totP, loP, hiP);
    tgtR = clampf(totR, -limR, limR);
    lastTgtP = tgtP;
    lastTgtR = tgtR;
  } else if (mode == MODE_MANUAL) {
    tgtP = clampf(manP, loP, hiP);
    tgtR = clampf(manR, -limR, limR);
    float ffb = safeTune() ? 0.0f : cfg.ffBody;  // so switching to stab starts exactly here (bumpless)
    cmdP = (1.0f - ffb) * tgtP;
    cmdR = (1.0f - ffb) * tgtR;
  } else if (mode == MODE_STAB) {  // stabilising but IMU lost / gyro calibration: hold where we are
    tgtP = lastTgtP;
    tgtR = lastTgtR;
  } else {  // centre / off
    cmdP = cmdR = 0;
  }
  if (mode != MODE_STAB) glInit = false;
  float rate = cfg.maxRate;
  if ((int32_t)(softStartUntil - millis()) > 0) rate = fminf(rate, 60.0f);
  profStep(slewP, velP, tgtP, rate, cfg.srvAcc, dt);
  profStep(slewR, velR, tgtR, rate, cfg.srvAcc, dt);
  lagP = tgtP - slewP;
  lagR = tgtR - slewR;
  float k = dt / (cfg.smooth + dt);
  outP += (slewP - outP) * k;
  outR += (slewR - outR) * k;
  srvModelStep(mdlP, outP, cfg.usDegP, dt);
  srvModelStep(mdlR, outR, cfg.usDegR, dt);
  float wantP = dbCompApply(dbcP, 1500 + cfg.trimP + cfg.dirP * outP * cfg.usDegP);
  float wantR = dbCompApply(dbcR, 1500 + cfg.trimR + cfg.dirR * outR * cfg.usDegR);
  // Pulse hold: sensor noise moved the pulse by a fraction of a us every 5 ms. A servo holding the D435
  // against gravity answers each tiny change with a kick of its motor: the constant buzz of the
  // atas-bawah servo. The pulse now only changes when the wanted value is 1.5 us (0.15 deg) away;
  // real motion passes straight through, noise does not.
  const float HOLD_US = 1.5f;
  if (mode != MODE_STAB || fabsf(wantP - usP) >= HOLD_US) usP = wantP;
  if (mode != MODE_STAB || fabsf(wantR - usR) >= HOLD_US) usR = wantR;
  if (mode == MODE_OFF) {
    pwmWriteUs(pinPitch(), 0);
    pwmWriteUs(pinRoll(), 0);
  } else {
    // Dither: +-ditherUs on alternate servo frames (50 Hz at 100 Hz pulses). The MG90S only moves once its
    // command has changed by more than its dead band and then jumps (measured: stalls, then 0.5..1.2 deg
    // jumps on a perfectly smooth ramp). The wobble is too fast and too small to move the camera, but it
    // keeps the motor from sticking, so small changes are followed at once: smooth instead of stepwise.
    float dz = 0.0f;
    if (cfg.ditherUs) dz = ((micros() / (1000000UL / cfg.pwmHz)) & 1) ? cfg.ditherUs : -(float)cfg.ditherUs;
    pwmWriteUs(pinPitch(), usP + dz);
    pwmWriteUs(pinRoll(), usR + dz);
  }
}

// ------------------------------------------------------------------------------------ WiFi + web app
// The gimbal joins the phone's hotspot (or, when it knows none, starts its own WiFi for the first
// setup) and serves the app itself: http://<name>.local or its IP. The app then opens a WebSocket on
// the same address and speaks the same text lines as USB. Up to 3 phones/tablets at once, each with
// the PIN. Everything here is non-blocking: sends take only what the network can take right now.
void handleLine(char *line, uint8_t src);
void newNonce(char *out) {
  for (int i = 0; i < 16; i++) sprintf(out + 2 * i, "%02x", (unsigned)(esp_random() & 0xFF));
}
WiFiServer webServer(80);
HttpConn http[HTTP_N];
char hostName[32] = "gimbal-d435", apSsid[33] = "Gimbal-D435";
bool wifiStarted = false, apOn = false, staUp = false;
uint32_t staDownSince = 0, staTryAt = 0, apOffAt = 0, wifiPollT = 0;
uint8_t scanDest = 0;

// --- SHA-1 and base64, only for the WebSocket handshake (RFC 6455)
void sha1(const uint8_t *d, size_t n, uint8_t out[20]) {
  uint32_t h[5] = { 0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0 };
  uint64_t bits = (uint64_t)n * 8;
  size_t total = ((n + 8) / 64 + 1) * 64;
  for (size_t off = 0; off < total; off += 64) {
    uint32_t w[80];
    for (int i = 0; i < 64; i++) {
      size_t p = off + i;
      uint8_t b = p < n ? d[p] : (p == n ? 0x80 : 0);
      if (p >= total - 8) b = (uint8_t)(bits >> (8 * (total - 1 - p)));
      if (i % 4 == 0) w[i / 4] = 0;
      w[i / 4] |= (uint32_t)b << (24 - 8 * (i % 4));
    }
    for (int i = 16; i < 80; i++) {
      uint32_t x = w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16];
      w[i] = (x << 1) | (x >> 31);
    }
    uint32_t a = h[0], b = h[1], c = h[2], e3 = h[3], e4 = h[4];
    for (int i = 0; i < 80; i++) {
      uint32_t f, k;
      if (i < 20) {
        f = (b & c) | (~b & e3);
        k = 0x5A827999;
      } else if (i < 40) {
        f = b ^ c ^ e3;
        k = 0x6ED9EBA1;
      } else if (i < 60) {
        f = (b & c) | (b & e3) | (c & e3);
        k = 0x8F1BBCDC;
      } else {
        f = b ^ c ^ e3;
        k = 0xCA62C1D6;
      }
      uint32_t t = ((a << 5) | (a >> 27)) + f + e4 + k + w[i];
      e4 = e3;
      e3 = c;
      c = (b << 30) | (b >> 2);
      b = a;
      a = t;
    }
    h[0] += a;
    h[1] += b;
    h[2] += c;
    h[3] += e3;
    h[4] += e4;
  }
  for (int i = 0; i < 20; i++) out[i] = (uint8_t)(h[i / 4] >> (24 - 8 * (i % 4)));
}
void base64(const uint8_t *in, size_t n, char *out) {
  static const char T[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  size_t j = 0;
  for (size_t i = 0; i < n; i += 3) {
    uint32_t v = (uint32_t)in[i] << 16 | (i + 1 < n ? (uint32_t)in[i + 1] << 8 : 0) | (i + 2 < n ? in[i + 2] : 0);
    out[j++] = T[(v >> 18) & 63];
    out[j++] = T[(v >> 12) & 63];
    out[j++] = i + 1 < n ? T[(v >> 6) & 63] : '=';
    out[j++] = i + 2 < n ? T[v & 63] : '=';
  }
  out[j] = 0;
}
void wsAcceptKey(const char *key, char out[32]) {
  char buf[100];
  snprintf(buf, sizeof(buf), "%s258EAFA5-E914-47DA-95CA-C5AB0DC85B11", key);
  uint8_t h[20];
  sha1((const uint8_t *)buf, strlen(buf), h);
  base64(h, 20, out);
}

// --- small text helpers
void hostSlug(const char *name, char *out, size_t cap) {  // "Gimbal D435" -> "gimbal-d435"
  size_t j = 0;
  bool dash = false;
  for (size_t i = 0; name[i] && j + 1 < cap; i++) {
    char c = tolower((unsigned char)name[i]);
    if (isalnum((unsigned char)c)) {
      if (dash && j) out[j++] = '-';
      if (j + 1 < cap) out[j++] = c;
      dash = false;
    } else dash = true;
  }
  out[j] = 0;
  if (!j) strncpy(out, "gimbal", cap);
}
void jsonEsc(const char *in, char *out, size_t cap) {
  size_t j = 0;
  for (size_t i = 0; in[i] && j + 7 < cap; i++) {
    unsigned char c = in[i];
    if (c == '"' || c == '\\') {
      out[j++] = '\\';
      out[j++] = c;
    } else if (c < 0x20) {
      j += snprintf(out + j, cap - j, "\\u%04x", c);
    } else out[j++] = c;
  }
  out[j] = 0;
}
bool hexDecode(const char *hex, char *out, size_t cap) {  // the app sends SSID/password as hex (spaces, symbols)
  if (!hex) {
    out[0] = 0;
    return true;
  }
  size_t n = strlen(hex);
  if (n % 2 || n / 2 >= cap) return false;
  for (size_t i = 0; i < n / 2; i++) {
    char b[3] = { hex[2 * i], hex[2 * i + 1], 0 };
    if (!isxdigit((unsigned char)b[0]) || !isxdigit((unsigned char)b[1])) return false;
    out[i] = (char)strtol(b, nullptr, 16);
  }
  out[n / 2] = 0;
  return true;
}
const char *hdrValue(const char *req, const char *name, char *out, size_t cap) {  // case-insensitive header
  size_t nl = strlen(name);
  for (const char *p = strstr(req, "\r\n"); p && p[2]; p = strstr(p + 2, "\r\n")) {
    const char *h = p + 2;
    if (strncasecmp(h, name, nl) == 0 && h[nl] == ':') {
      h += nl + 1;
      while (*h == ' ') h++;
      size_t j = 0;
      while (h[j] && h[j] != '\r' && j + 1 < cap) {
        out[j] = h[j];
        j++;
      }
      out[j] = 0;
      return out;
    }
  }
  out[0] = 0;
  return nullptr;
}
int netSend(WiFiClient &c, const void *b, size_t n) {  // never waits: bytes taken (maybe 0), -1 = gone
  if (!n) return 0;
  int r = send(c.fd(), b, n, MSG_DONTWAIT);
  if (r >= 0) return r;
  return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
}

// --- WiFi status
int wifiRssi() {
  return staUp ? WiFi.RSSI() : 0;
}
void wifiStatus(uint8_t dest) {
  char ss[70], ip[16] = "", apip[16] = "";
  jsonEsc(cfg.wifiSsid, ss, sizeof(ss));
  if (staUp) strncpy(ip, WiFi.localIP().toString().c_str(), sizeof(ip) - 1);
  if (apOn) strncpy(apip, WiFi.softAPIP().toString().c_str(), sizeof(apip) - 1);
  sendf(dest,
        "WIFI {\"on\":%d,\"sta\":%d,\"ssid\":\"%s\",\"ip\":\"%s\",\"rssi\":%d,\"ap\":%d,\"apssid\":\"%s\",\"apip\":\"%s\","
        "\"apn\":%d,\"host\":\"%s\",\"clients\":%d,\"saved\":%d}",
        cfg.wifiOn, staUp ? 1 : 0, ss, ip, wifiRssi(), apOn ? 1 : 0, apSsid, apip, apOn ? (int)WiFi.softAPgetStationNum() : 0,
        hostName, wsCount(true), cfg.wifiSsid[0] ? 1 : 0);
}
void apStart() {
  if (apOn) return;
  WiFi.softAP(apSsid, cfg.apPass, 1, 0, 4);
  apOn = true;
  event("WiFi gimbal aktif: %s (sandi %s). Buka http://192.168.4.1", apSsid, cfg.apPass);
}
void apStop() {
  if (!apOn) return;
  WiFi.softAPdisconnect(true);
  apOn = false;
  event("WiFi gimbal dinonaktifkan; memakai hotspot %s", cfg.wifiSsid);
}
void staTry() {
  staTryAt = millis();
  WiFi.begin(cfg.wifiSsid, cfg.wifiPass);
}
void wifiInit() {
  if (!cfg.wifiOn) {
    WiFi.mode(WIFI_OFF);
    return;
  }
  uint8_t mac[6];
  WiFi.mode(WIFI_AP_STA);
  WiFi.macAddress(mac);
  snprintf(apSsid, sizeof(apSsid), "Gimbal-D435-%02X%02X", mac[4], mac[5]);  // unique: two gimbals never clash
  hostSlug(cfg.name, hostName, sizeof(hostName));
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);  // retried by wifiPoll, so the setup WiFi stays steady
  WiFi.setHostname(hostName);
  WiFi.setSleep(false);                // answers within milliseconds instead of ~100 ms
  WiFi.setTxPower(WIFI_POWER_8_5dBm);  // less current and heat; also steadier on the C3 Super Mini antenna
  staDownSince = millis();
  if (cfg.wifiSsid[0]) staTry();
  else apStart();
  webServer.begin();
  webServer.setNoDelay(true);
  if (MDNS.begin(hostName)) MDNS.addService("http", "tcp", 80);
  for (int i = 0; i < WS_N; i++) newNonce(ws[i].nonce);
  wifiStarted = true;
}
// every 250 ms: follow the hotspot, keep the setup WiFi on only while it is needed
void wifiPoll() {
  uint32_t now = millis();
  if (!wifiStarted || (uint32_t)(now - wifiPollT) < 250) return;
  wifiPollT = now;
  bool up = WiFi.status() == WL_CONNECTED;
  if (up && !staUp) {
    staUp = true;
    event("Terhubung ke %s. Buka http://%s.local atau http://%s", cfg.wifiSsid, hostName, WiFi.localIP().toString().c_str());
    if (apOn) apOffAt = now + (WiFi.softAPgetStationNum() ? 120000 : 5000);  // let the setup page show the new address
    wifiStatus(allDest());
  } else if (!up && staUp) {
    staUp = false;
    staDownSince = now;
    event("Hotspot %s terputus; mencoba menghubungkan kembali", cfg.wifiSsid);
    wifiStatus(allDest());
  }
  if (staUp && apOn && apOffAt && (int32_t)(now - apOffAt) >= 0) {
    apOffAt = 0;
    apStop();
    WiFi.mode(WIFI_STA);
  }
  if (!staUp) {
    apOffAt = 0;
    if (!apOn && (uint32_t)(now - staDownSince) > 20000) {  // hotspot not found: open the setup WiFi
      WiFi.mode(WIFI_AP_STA);
      apStart();
    }
    // retry the hotspot every 15 s, but not while someone is on the setup WiFi (a retry scans all
    // channels and would make their connection drop)
    if (cfg.wifiSsid[0] && (uint32_t)(now - staTryAt) > 15000 && !(apOn && WiFi.softAPgetStationNum() > 0)) staTry();
  }
  if (scanDest) {
    int n = WiFi.scanComplete();
    if (n >= 0 || n == -2) {
      static char out[1000];
      size_t j = snprintf(out, sizeof(out), "WIFI LIST [");
      int shown = 0;
      for (int i = 0; i < n && shown < 15 && j < sizeof(out) - 90; i++) {
        String nm = WiFi.SSID(i);
        if (!nm.length()) continue;
        bool dup = false;  // the same hotspot seen twice (two bands): keep the first
        for (int k = 0; k < i && !dup; k++) dup = WiFi.SSID(k) == nm;
        if (dup) continue;
        char e[70];
        jsonEsc(nm.c_str(), e, sizeof(e));
        j += snprintf(out + j, sizeof(out) - j, "%s{\"s\":\"%s\",\"r\":%d,\"k\":%d}", shown ? "," : "", e, (int)WiFi.RSSI(i),
                      WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? 0 : 1);
        shown++;
      }
      snprintf(out + j, sizeof(out) - j, "]");
      sendTo(scanDest, out);
      WiFi.scanDelete();
      scanDest = 0;
    }
  }
}
void wifiCommand(char *a1, char *a2, char *a3, uint8_t src) {
  char sub[12] = "";
  if (a1) {
    strncpy(sub, a1, sizeof(sub) - 1);
    for (char *p = sub; *p; p++) *p = toupper(*p);
  }
  if (!sub[0]) {
    wifiStatus(src);
  } else if (!strcmp(sub, "SCAN")) {
    if (!wifiStarted) return (void)sendTo(src, "ERR WiFi nonaktif");
    WiFi.scanNetworks(true, false, false, 120);
    scanDest = src;
    sendTo(src, "OK WIFI SCAN");
  } else if (!strcmp(sub, "JOIN")) {
    char ss[33], pw[65];
    if (!hexDecode(a2, ss, sizeof(ss)) || !hexDecode(a3, pw, sizeof(pw)) || !ss[0]) {
      sendTo(src, "ERR WIFI JOIN <nama-hex> <sandi-hex>");
      return;
    }
    if (pw[0] && strlen(pw) < 8) {
      sendTo(src, "ERR sandi hotspot minimal 8 karakter");
      return;
    }
    strcpy(cfg.wifiSsid, ss);
    strcpy(cfg.wifiPass, pw);
    cfgSave();
    sendTo(src, "OK WIFI JOIN");
    event("Menghubungkan ke hotspot %s...", ss);
    if (wifiStarted) {
      staDownSince = millis();
      staTry();
    }
  } else if (!strcmp(sub, "FORGET")) {
    cfg.wifiSsid[0] = cfg.wifiPass[0] = 0;
    cfgSave();
    sendTo(src, "OK WIFI FORGET");
    if (wifiStarted) {
      WiFi.disconnect(false);
      WiFi.mode(WIFI_AP_STA);
      apStart();
    }
  } else if (!strcmp(sub, "AP")) {
    char pw[33];
    if (!hexDecode(a2, pw, sizeof(pw)) || strlen(pw) < 8) {
      sendTo(src, "ERR sandi WiFi gimbal 8-32 karakter");
      return;
    }
    strcpy(cfg.apPass, pw);
    cfgSave();
    sendTo(src, "OK WIFI AP (berlaku saat WiFi gimbal aktif kembali)");
  } else if (!strcmp(sub, "APOFF")) {
    if (!staUp) return (void)sendTo(src, "ERR belum terhubung ke hotspot; WiFi gimbal tetap aktif");
    apOffAt = millis() + 1500;
    sendTo(src, "OK WIFI APOFF");
  } else if (!strcmp(sub, "ON") || !strcmp(sub, "OFF")) {
    if (src != TO_USB) return (void)sendTo(src, "ERR WIFI ON/OFF hanya dapat dikirim lewat kabel USB");
    cfg.wifiOn = !strcmp(sub, "ON");
    cfgSave();
    sendTo(src, "OK, memulai ulang...");
    rebootAt = millis() + 300;
  } else {
    sendTo(src, "ERR WIFI | WIFI SCAN | WIFI JOIN | WIFI FORGET | WIFI AP | WIFI APOFF");
  }
  (void)a3;
}

// --- HTTP: the app's files from flash, gzip, revalidated with ETag
void wsClose(int i) {
  WsConn &w = ws[i];
  bool was = w.authed;
  w.c.stop();
  w.c = WiFiClient();
  w.used = w.authed = false;
  w.rn = w.ln = w.th = w.tt = w.frN = w.frOff = 0;
  w.closeAt = 0;
  newNonce(w.nonce);
  if (was) event("Aplikasi terputus (%d perangkat masih terhubung)", wsCount(true));
}
void httpClose(HttpConn &h) {
  h.c.stop();
  h.c = WiFiClient();
  h.used = false;
}
void httpReply(HttpConn &h, const char *status, const char *extra, const uint8_t *body, uint32_t len) {
  h.hn = snprintf(h.head, sizeof(h.head), "HTTP/1.1 %s\r\n%sContent-Length: %lu\r\nConnection: close\r\n\r\n", status, extra,
                  (unsigned long)len);
  h.hoff = 0;
  h.body = body;
  h.len = len;
  h.off = 0;
}
void httpRequest(HttpConn &h) {
  char method[8] = "", path[96] = "", v[80];
  sscanf(h.req, "%7s %95s", method, path);
  char *q = strchr(path, '?');
  if (q) *q = 0;
  if (!strcmp(path, "/ws") && hdrValue(h.req, "Upgrade", v, sizeof(v)) && strcasecmp(v, "websocket") == 0 &&
      hdrValue(h.req, "Sec-WebSocket-Key", v, sizeof(v))) {
    int slot = -1;
    for (int i = 0; i < WS_N && slot < 0; i++)
      if (!ws[i].used) slot = i;
    if (slot < 0) {
      static const char msg[] = "Sudah 3 perangkat tersambung";
      httpReply(h, "503 Service Unavailable", "Content-Type: text/plain\r\n", (const uint8_t *)msg, sizeof(msg) - 1);
      return;
    }
    WsConn &w = ws[slot];
    char acc[32];
    wsAcceptKey(v, acc);
    w.c = h.c;  // the connection now belongs to the WebSocket
    h.c = WiFiClient();
    h.used = false;
    w.used = true;
    w.authed = false;
    w.rn = w.ln = w.th = w.tt = 0;
    w.closeAt = 0;
    w.lastRx = millis();
    w.frN = snprintf((char *)w.fr, sizeof(w.fr),
                     "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: %s\r\n\r\n", acc);
    w.frOff = 0;
    return;
  }
  if (strcmp(method, "GET") && strcmp(method, "HEAD")) {
    httpReply(h, "405 Method Not Allowed", "", nullptr, 0);
    return;
  }
  if (!strcmp(path, "/")) strcpy(path, "/index.html");
  for (int i = 0; i < WEB_FILES_N; i++) {
    const WebFile &f = WEB_FILES[i];
    if (strcmp(path, f.path)) continue;
    char extra[200];
    snprintf(extra, sizeof(extra), "Content-Type: %s\r\nContent-Encoding: gzip\r\nCache-Control: no-cache\r\nETag: %s\r\n", f.type, f.etag);
    if (hdrValue(h.req, "If-None-Match", v, sizeof(v)) && !strcmp(v, f.etag)) {
      snprintf(extra, sizeof(extra), "ETag: %s\r\n", f.etag);
      httpReply(h, "304 Not Modified", extra, nullptr, 0);
    } else {
      httpReply(h, "200 OK", extra, f.data, f.len);
      if (!strcmp(method, "HEAD")) h.len = 0;
    }
    return;
  }
  static const char nf[] = "Tidak ada";
  httpReply(h, "404 Not Found", "Content-Type: text/plain\r\n", (const uint8_t *)nf, sizeof(nf) - 1);
}
void httpPoll(HttpConn &h) {
  if (!h.used) return;
  if ((uint32_t)(millis() - h.t0) > 15000 || (!h.c.connected() && !h.c.available())) {
    httpClose(h);
    return;
  }
  if (!h.hn) {  // still reading the request
    while (h.c.available() && h.rn < sizeof(h.req) - 1) h.req[h.rn++] = (char)h.c.read();
    h.req[h.rn] = 0;
    if (strstr(h.req, "\r\n\r\n") || h.rn >= sizeof(h.req) - 1) {
      httpRequest(h);
      if (!h.used) return;  // became a WebSocket
    } else return;
  }
  if (h.hoff < h.hn) {
    int r = netSend(h.c, h.head + h.hoff, h.hn - h.hoff);
    if (r < 0) return httpClose(h);
    h.hoff += r;
    if (h.hoff < h.hn) return;
  }
  if (h.off < h.len) {
    size_t n = h.len - h.off;
    if (n > 2920) n = 2920;
    int r = netSend(h.c, h.body + h.off, n);
    if (r < 0) return httpClose(h);
    h.off += r;
  }
  if (h.off >= h.len) httpClose(h);
}

// --- WebSocket: one text frame per command line in, frames of whole text out
void wsPoll(int i) {
  WsConn &w = ws[i];
  if (!w.used) return;
  uint32_t now = millis();
  if ((w.closeAt && (int32_t)(now - w.closeAt) >= 0) || (uint32_t)(now - w.lastRx) > 20000 ||
      (!w.c.connected() && !w.c.available())) {  // the app pings every 5 s; 20 s of silence = gone
    wsClose(i);
    return;
  }
  while (w.c.available() && w.rn < sizeof(w.rx)) w.rx[w.rn++] = (uint8_t)w.c.read();
  while (w.used && w.rn >= 2) {
    uint8_t op = w.rx[0] & 0x0F;
    bool fin = w.rx[0] & 0x80, masked = w.rx[1] & 0x80;
    uint32_t len = w.rx[1] & 0x7F;
    size_t hl = 2;
    if (len == 126) {
      if (w.rn < 4) break;
      len = (uint32_t)w.rx[2] << 8 | w.rx[3];
      hl = 4;
    } else if (len == 127) {
      wsClose(i);
      return;
    }
    if (masked) hl += 4;
    if (hl + len > sizeof(w.rx)) {  // commands are short; anything this big is not from the app
      wsClose(i);
      return;
    }
    if (w.rn < hl + len) break;
    uint8_t *pl = w.rx + hl;
    if (masked)
      for (uint32_t k = 0; k < len; k++) pl[k] ^= w.rx[hl - 4 + (k & 3)];
    w.lastRx = now;
    if (op == 0x8) {
      wsClose(i);
      return;
    }
    if (op == 0x1 || op == 0x0) {
      for (uint32_t k = 0; k <= len; k++) {
        bool end = (k == len) ? fin : (pl[k] == '\n');
        if (k < len && pl[k] != '\n' && pl[k] != '\r' && w.ln < sizeof(w.line) - 1) w.line[w.ln++] = (char)pl[k];
        if (end && w.ln) {
          w.line[w.ln] = 0;
          w.ln = 0;
          handleLine(w.line, toWs(i));
          if (!w.used) return;
        }
      }
    }
    size_t used = hl + len;
    memmove(w.rx, w.rx + used, w.rn - used);
    w.rn -= used;
  }
  // send: finish the frame in flight, then pack waiting text into the next one
  for (int k = 0; k < 4 && w.used; k++) {
    if (w.frOff >= w.frN) {
      size_t n = wsUsed(w);
      if (!n) break;
      if (n > sizeof(w.fr) - 4) n = sizeof(w.fr) - 4;
      size_t hl = 2;
      w.fr[0] = 0x81;
      if (n < 126) w.fr[1] = (uint8_t)n;
      else {
        w.fr[1] = 126;
        w.fr[2] = (uint8_t)(n >> 8);
        w.fr[3] = (uint8_t)n;
        hl = 4;
      }
      for (size_t j = 0; j < n; j++) {
        w.fr[hl + j] = (uint8_t)w.tx[w.tt];
        w.tt = (w.tt + 1) % WS_TX;
      }
      w.frN = hl + n;
      w.frOff = 0;
    }
    int r = netSend(w.c, w.fr + w.frOff, w.frN - w.frOff);
    if (r < 0) {
      wsClose(i);
      return;
    }
    w.frOff += r;
    if (w.frOff < w.frN) break;  // network full: rest next time
  }
}
void netPoll() {
  if (!wifiStarted) return;
  wifiPoll();
  WiFiClient c = webServer.accept();
  if (c) {
    int slot = -1;
    for (int i = 0; i < HTTP_N && slot < 0; i++)
      if (!http[i].used) slot = i;
    if (slot < 0) c.stop();  // the browser will retry
    else {
      HttpConn &h = http[slot];
      h.c = c;
      h.c.setNoDelay(true);
      h.used = true;
      h.rn = h.hn = h.hoff = 0;
      h.req[0] = 0;
      h.len = h.off = 0;
      h.t0 = millis();
    }
  }
  for (int i = 0; i < HTTP_N; i++) httpPoll(http[i]);
  for (int i = 0; i < WS_N; i++) wsPoll(i);
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
int wsIndex(uint8_t src) {  // which web client a single-bit destination is, -1 for USB
  for (int i = 0; i < WS_N; i++)
    if (src == toWs(i)) return i;
  return -1;
}
void sendHello(uint8_t dest) {
  int i = wsIndex(dest);
  bool needAuth = i >= 0 && !ws[i].authed;
  long lock = 0;
  if ((int32_t)(lockUntil - millis()) > 0) lock = (lockUntil - millis()) / 1000 + 1;
  sendf(dest, "HELLO {\"fw\":\"%s\",\"name\":\"%s\",\"auth\":%d,\"nonce\":\"%s\",\"lock\":%ld,\"clients\":%d}", FW_VERSION,
        cfg.name, needAuth ? 1 : 0, i >= 0 ? ws[i].nonce : "", lock, wsCount(false));
}
void sendCfg(uint8_t dest) {
  char ax[8];
  axesString(ax, cfg.axF, cfg.sgF, cfg.axL, cfg.sgL, cfg.axU, cfg.sgU);
  sendf(dest,
        "CFG {\"pitch\":%.2f,\"roll\":%.2f,\"kp\":%.2f,\"gain\":%.2f,\"ff\":%.2f,\"db\":%.2f,\"smooth\":%.3f,\"maxrate\":%.0f,"
        "\"acctau\":%.2f,\"dlpf\":%u,\"autobias\":%u,\"bootgyro\":%u,\"nod\":%u,\"trimp\":%d,\"trimr\":%d,\"dirp\":%d,"
        "\"dirr\":%d,\"swap\":%d,\"usdegp\":%.2f,\"usdegr\":%.2f,\"limp\":%.1f,\"limr\":%.1f,\"lvlp\":%.2f,\"lvlr\":%.2f,"
        "\"axes\":\"%s\",\"tel\":%d,\"led\":%d,\"startmode\":%d,\"name\":\"%s\",\"cal\":%u,\"defpin\":%d,\"mode\":\"%s\","
        "\"ffb\":%.2f,\"pred\":%.2f,\"sdelay\":%.1f,\"stau\":%.1f,\"acctauw\":%.2f,\"dbcomp\":%.1f,\"sdb\":%.1f,\"pwmhz\":%u,\"gait\":%.2f,\"soft\":%.2f,\"srvacc\":%.0f,\"limpdn\":%.0f,\"limpup\":%.0f,\"twinp\":%.2f,\"twinr\":%.2f,\"dither\":%u}",
        cfg.pitchSet, cfg.rollSet, cfg.kp, cfg.gain, cfg.rateFF, cfg.deadband, cfg.smooth, cfg.maxRate, cfg.accTau, cfg.dlpf,
        cfg.autoBias, cfg.bootGyro, cfg.nod, cfg.trimP, cfg.trimR, cfg.dirP, cfg.dirR, cfg.swapServo, cfg.usDegP,
        cfg.usDegR, cfg.limP, cfg.limR, cfg.lvlP, cfg.lvlR, ax, cfg.telHz, cfg.ledOn, cfg.startMode, cfg.name, cfg.calOK,
        strcmp(cfg.pin, DEFAULT_PIN) == 0 ? 1 : 0, MODE_NAMES[mode], cfg.ffBody, cfg.pred, cfg.srvDelay, cfg.srvTau,
        cfg.accTauW, cfg.dbComp, cfg.srvDb, (unsigned)cfg.pwmHz, cfg.gait, cfg.soft, cfg.srvAcc, cfg.limPUsDn, cfg.limPUsUp, cfg.twinP, cfg.twinR, (unsigned)cfg.ditherUs);
}
void sendStatus(uint8_t dest) {
  sendf(dest,
        "ST {\"fw\":\"%s\",\"up\":%lu,\"hz\":%lu,\"rst\":\"%s\",\"boots\":%lu,\"brown\":%lu,\"imu\":%d,\"who\":\"0x%02X\","
        "\"addr\":\"0x%02X\",\"i2cerr\":%lu,\"temp\":%.1f,\"bias\":[%.2f,%.2f,%.2f],\"heap\":%lu,\"clients\":%d,\"rssi\":%d,"
        "\"mode\":\"%s\",\"cal\":\"%s\",\"still\":%d,\"accrej\":%.0f,\"usp\":%.0f,\"usr\":%.0f,\"swap\":%d,"
        "\"walk\":%.2f,\"ffp\":%.2f,\"ffr\":%.2f,\"ip\":%.2f,\"ir\":%.2f,\"stride\":%.2f,\"sway\":%.1f,"
        "\"gaitp\":%.2f,\"cpu\":%lu,\"cpumax\":%lu}",
        FW_VERSION, (unsigned long)(millis() / 1000), (unsigned long)loopHz, resetReasonStr(rstReason),
        (unsigned long)bootCount, (unsigned long)brownCount, imuOK ? 1 : 0, imuWho, imuAddr, (unsigned long)i2cErr,
        temperatureRead(), cfg.bias[0], cfg.bias[1], cfg.bias[2], (unsigned long)ESP.getFreeHeap(),
        wsCount(true), wifiRssi(), MODE_NAMES[mode], CAL_NAMES[cal.kind], isStill ? 1 : 0,
        accRejPct, usP, usR, cfg.swapServo, walkF, ffP, ffR, cmdP, cmdR, gait.w / (2.0f * (float)M_PI), gait.amp,
        gaitAdd(), (unsigned long)cpuUs, (unsigned long)cpuMaxUs);
}
void sendHelp(uint8_t dest) {
  static const char *lines[] = {
    "HELP perintah:",
    "HELP GET | STATUS | EVLOG | PING | HELLO",
    "HELP SET <kunci> <nilai>   (pitch roll kp gain ff db smooth maxrate acctau dlpf autobias bootgyro nod",
    "HELP     trimp trimr dirp dirr swap usdegp usdegr limp limr lvlp lvlr axes tel led startmode",
    "HELP     ffb pred sdelay stau sdb dbcomp acctauw pwmhz gait soft srvacc limpdn limpup twinp twinr dither)",
    "HELP MODE stab|center|manual|off   MAN <atas-bawah> <kanan-kiri>   LIMP down|up (batas = posisi Manual sekarang)",
    "HELP CAL gyro | CAL level [atas-bawah|x] [kanan-kiri|x] | CAL twin | CAL abort",
    "HELP ACC | ACC off <x> <y> <z> | ACC gain <x> <y> <z> | ACC reset",
    "HELP STEP p|r <derajat>   SAVE   DEFAULTS tuning|all   PIN <baru>   NAME <nama>   REBOOT",
    "HELP WIFI | WIFI SCAN | WIFI JOIN <ssid-hex> <sandi-hex> | WIFI FORGET | WIFI AP <sandi-hex> | WIFI APOFF | WIFI ON|OFF",
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
  if (!strcmp(key, "maxrate")) {  // servo speed (deg/s): the user's choice, slow by default
    cfg.maxRate = clampf(v, 20.0f, 600.0f);
    if (cfg.maxRate > SPEED_WARN)
      event("Peringatan: kecepatan servo %.0f derajat/detik. Di atas %.0f servo MG90S cepat panas dan giginya cepat aus.", cfg.maxRate,
            SPEED_WARN);
    return true;
  }
  F_("pitch", pitchSet, -45.0f, 60.0f)  // negative = camera looks up
  F_("roll", rollSet, -15.0f, 15.0f)
  F_("kp", kp, 0.0f, 1.5f)
  F_("gain", gain, 0.5f, 30.0f)
  F_("ff", rateFF, 0.0f, 1.5f)
  F_("db", deadband, 0.0f, 2.0f)
  F_("smooth", smooth, 0.0f, 0.3f)
  if (!strcmp(key, "srvacc")) {  // optional servo acceleration limit (deg/s^2), 0 = off
    cfg.srvAcc = v <= 0.0f ? 0.0f : clampf(v, 200.0f, 20000.0f);
    return true;
  }
  F_("acctau", accTau, 0.1f, 5.0f)
  F_("ffb", ffBody, 0.0f, 1.0f)
  F_("pred", pred, 0.0f, 1.5f)
  F_("sdelay", srvDelay, 0.0f, 60.0f)
  F_("stau", srvTau, 5.0f, 120.0f)
  F_("acctauw", accTauW, 0.3f, 10.0f)
  F_("dbcomp", dbComp, 0.0f, 60.0f)  // > 10: also covers gear/horn play (backlash) measured on the frame
  F_("sdb", srvDb, 0.0f, 40.0f)
  F_("gait", gait, 0.0f, 1.0f)
  F_("soft", soft, 0.0f, 1.0f)
  I_("autobias", autoBias, 0, 1)
  I_("bootgyro", bootGyro, 0, 1)
  I_("nod", nod, 0, 1)
  I_("trimp", trimP, -400, 400)
  I_("trimr", trimR, -400, 400)
  I_("swap", swapServo, 0, 1)
  F_("usdegp", usDegP, 5.0f, 60.0f)
  F_("usdegr", usDegR, 5.0f, 60.0f)
  F_("limp", limP, 3.0f, 60.0f)
  F_("limpdn", limPUsDn, -1000.0f, 1000.0f)
  F_("limpup", limPUsUp, -1000.0f, 1000.0f)
  F_("twinp", twinP, -89.0f, 89.0f)
  F_("twinr", twinR, -89.0f, 89.0f)
  F_("limr", limR, 3.0f, 60.0f)
  F_("lvlp", lvlP, -30.0f, 30.0f)
  F_("lvlr", lvlR, -30.0f, 30.0f)
  I_("tel", telHz, 0, 25)
  I_("dither", ditherUs, 0, 20)
  I_("led", ledOn, 0, 1)
  I_("startmode", startMode, 0, 3)
#undef F_
#undef I_
  if (!strcmp(key, "pwmhz")) {
    cfg.pwmHz = (uint16_t)clampf((float)iv, 50, 333);
    pwmSetHz(cfg.pwmHz);
    return true;
  }
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
void handleAuth(const char *hex, uint8_t src) {
  int wi = wsIndex(src);
  if (wi < 0) {  // USB: the cable itself is the key
    sendTo(src, "AUTH OK");
    return;
  }
  WsConn &w = ws[wi];
  uint32_t now = millis();
  if ((int32_t)(lockUntil - now) > 0) {
    sendf(src, "AUTH LOCKED %lu", (unsigned long)((lockUntil - now) / 1000 + 1));
    return;
  }
  char expect[65];
  authHash(w.nonce, cfg.pin, expect);
  if (hex && strlen(hex) == 64 && strcasecmp(hex, expect) == 0) {
    w.authed = true;
    authFails = 0;
    sendTo(src, "AUTH OK");
    sendCfg(src);
    event("Aplikasi terhubung lewat WiFi, sandi benar (%d perangkat)", wsCount(true));
    if (cfg.nod && mode == MODE_STAB) {  // small nod: physical "connected" sign
      nodOffset = 4.0f;
      nodUntil = millis() + 400;
      if (nodUntil == 0) nodUntil = 1;
    }
  } else {
    authFails++;
    newNonce(w.nonce);
    if (authFails >= 5) {
      authFails = 0;
      lockUntil = now + 30000;
      sendTo(src, "AUTH LOCKED 30");
      event("Sandi salah 5 kali lewat WiFi: dikunci 30 detik");
      w.closeAt = now + 400;
      if (w.closeAt == 0) w.closeAt = 1;
    } else {
      sendf(src, "AUTH FAIL %d %s", 5 - authFails, w.nonce);
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
  int wi = wsIndex(src);
  bool authed = wi < 0 || ws[wi].authed;
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
    if (m == MODE_STAB && mode != MODE_STAB) {  // continue smoothly from where the servos are
      float ffb = safeTune() ? 0.0f : cfg.ffBody;
      cmdP = (1.0f - ffb) * outP;
      cmdR = (1.0f - ffb) * outR;
      glInit = false;  // the target glides from where the camera points now
    }
    mode = (Mode)m;
    sendCfg(allDest() | src);
  } else if (!strcmp(cmd, "LIMP")) {  // LIMP down|up: the atas-bawah servo's current position becomes that limit
    if (mode != MODE_MANUAL || !a1 || (strcasecmp(a1, "down") && strcasecmp(a1, "up"))) {
      sendTo(src, "ERR LIMP down|up, dalam MODE manual: geser servo atas-bawah ke posisi aman terjauh dulu");
      return;
    }
    float off = usP - 1500.0f - cfg.trimP;  // the pulse the servo has now, from its centre
    bool dn = !strcasecmp(a1, "down");
    if (dn) cfg.limPUsDn = off;
    else cfg.limPUsUp = off;
    cfgSaveSoon();
    sendf(src, "OK LIMP %s %.0f us", dn ? "down" : "up", off);
    event("Batas servo atas-bawah %s: %.0f us dari tengah (%.1f derajat poros MG90S)", dn ? "menunduk" : "mendongak", off, off / 11.1f);
    sendCfg(allDest() | src);
  } else if (!strcmp(cmd, "MAN")) {
    if (!a1 || !a2) {
      sendTo(src, "ERR MAN <atas-bawah> <kanan-kiri>");
      return;
    }
    if (mode != MODE_MANUAL) {
      sendTo(src, "ERR pindah ke MODE manual terlebih dahulu");
      return;
    }
    manP = clampf(atof(a1), -cfg.limP, cfg.limP);
    manR = clampf(atof(a2), -cfg.limR, cfg.limR);
    sendf(src, "OK MAN %.2f %.2f", manP, manR);
  } else if (!strcmp(cmd, "CAL")) {
    if (!a1) {
      sendTo(src, "ERR CAL gyro|level|twin|abort");
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
      sendTo(src, "ERR IMU tidak terbaca; periksa keempat kabel IMU");
      return;
    }
    if (!strcmp(a1, "gyro")) {
      calStart(CAL_GYRO, src);
    } else if (!strcmp(a1, "level")) {
      // CAL level <atas-bawah terbaca|x> <kanan-kiri terbaca|x>   (x = jangan ubah sumbu itu)
      calStart(CAL_LEVEL, src);
      cal.useP = !(a2 && tolower(a2[0]) == 'x');
      cal.useR = !(a3 && tolower(a3[0]) == 'x');
      cal.refP = (a2 && cal.useP) ? atof(a2) : 0.0f;
      cal.refR = (a3 && cal.useR) ? atof(a3) : 0.0f;
    } else if (!strcmp(a1, "auto")) {
      sendTo(src, "ERR kalibrasi servo sudah dihapus: atur arah, kabel, dan skala servo secara manual (SET dirp/dirr/swap/usdegp/usdegr)");
    } else if (!strcmp(a1, "twin")) {
      // 3D model (digital twin): where the camera points with both servos in the centre and the body
      // upright = what the IMU reads now minus what the servos add now. No servo moves for this.
      if (mode == MODE_OFF) {
        sendTo(src, "CAL FAIL twin servo nonaktif: pilih mode Tengah, lalu tegakkan badan gimbal");
        return;
      }
      if (!isStill) {
        sendTo(src, "CAL FAIL twin gimbal belum diam: letakkan tegak dan jangan disentuh sebentar");
        return;
      }
      cfg.twinP = clampf(pitchMeas - outP, -89.0f, 89.0f);
      cfg.twinR = clampf(rollMeas - outR, -89.0f, 89.0f);
      cfg.calOK |= 8;
      cfgSaveSoon();
      sendf(allDest() | src, "CAL DONE twin {\"twinp\":%.2f,\"twinr\":%.2f}", cfg.twinP, cfg.twinR);
      event("Model 3D disamakan: kamera %.1f derajat atas-bawah, %.1f derajat kanan-kiri saat servo di tengah", cfg.twinP, cfg.twinR);
      sendCfg(allDest() | src);
    } else sendTo(src, "ERR CAL gyro|level|twin|abort");
  } else if (!strcmp(cmd, "GYRO")) {  // GYRO | GYRO reset: rotation per raw gyro axis, to check each axis
    if (a1 && !strcasecmp(a1, "reset")) {
      for (int k = 0; k < 3; k++) gyrInt[k] = 0;
      sendTo(src, "OK GYRO reset");
    } else {
      sendf(src, "GYRO int %.2f %.2f %.2f now %.2f %.2f %.2f bias %.2f %.2f %.2f", gyrInt[0], gyrInt[1], gyrInt[2], gyrRaw[0], gyrRaw[1],
            gyrRaw[2], cfg.bias[0], cfg.bias[1], cfg.bias[2]);
    }
  } else if (!strcmp(cmd, "ACC")) {  // ACC | ACC off <x> <y> <z> | ACC gain <x> <y> <z> | ACC reset
    if (a1 && (!strcmp(a1, "off") || !strcmp(a1, "gain"))) {
      char *a4 = strtok_r(nullptr, " ", &save);
      if (!a2 || !a3 || !a4) {
        sendTo(src, "ERR ACC off|gain <x> <y> <z>");
        return;
      }
      float v[3] = { (float)atof(a2), (float)atof(a3), (float)atof(a4) };
      bool off = !strcmp(a1, "off");
      for (int k = 0; k < 3; k++) {
        if (off ? !(fabsf(v[k]) <= 0.5f) : !(v[k] >= 0.7f && v[k] <= 1.4f)) {
          sendTo(src, off ? "ERR offset harus -0.5..0.5 g" : "ERR gain harus 0.7..1.4");
          return;
        }
      }
      float *dst = off ? accOff : accGain;
      for (int k = 0; k < 3; k++) dst[k] = v[k];
      prefs.putBytes(off ? "accoff" : "accgain", dst, 3 * sizeof(float));
      attitudeReset();
    } else if (a1 && !strcmp(a1, "reset")) {
      for (int k = 0; k < 3; k++) {
        accOff[k] = 0;
        accGain[k] = 1;
      }
      prefs.putBytes("accoff", accOff, sizeof(accOff));
      prefs.putBytes("accgain", accGain, sizeof(accGain));
      attitudeReset();
    } else if (a1) {
      sendTo(src, "ERR ACC | ACC off <x> <y> <z> | ACC gain <x> <y> <z> | ACC reset");
      return;
    }
    float n = 0;  // |a| of the averaged reading with the current offset and gain: 1.000 when right
    for (int k = 0; k < 3; k++) {
      float c = (accAvgRaw[k] - accOff[k]) * accGain[k];
      n += c * c;
    }
    n = sqrtf(n);
    sendf(src, "ACC raw %.4f %.4f %.4f off %.4f %.4f %.4f gain %.4f %.4f %.4f norm %.4f", accAvgRaw[0], accAvgRaw[1],
          accAvgRaw[2], accOff[0], accOff[1], accOff[2], accGain[0], accGain[1], accGain[2], n);
  } else if (!strcmp(cmd, "STEP")) {
    if (mode != MODE_STAB || !imuOK || cal.kind != CAL_IDLE || capState) {
      sendTo(src, "ERR uji respons hanya dapat dijalankan dalam mode stab, saat tidak ada kalibrasi");
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
      cfg.srvDelay = keep.srvDelay;  // measured by CAL auto
      cfg.srvTau = keep.srvTau;
      cfg.srvDb = keep.srvDb;
      memcpy(cfg.bias, keep.bias, sizeof(cfg.bias));
    }
    cfgSave();
    attitudeReset();
    event("Pengaturan dikembalikan ke nilai bawaan (%s)", all ? "semua" : "tuning");
    sendCfg(allDest() | src);
  } else if (!strcmp(cmd, "FACTORY")) {
    if (src != TO_USB) {
      sendTo(src, "ERR FACTORY hanya lewat kabel USB");
      return;
    }
    cfgDefaults(cfg, false);
    cfgSave();
    sendTo(src, "OK FACTORY, memulai ulang...");
    rebootAt = millis() + 300;
  } else if (!strcmp(cmd, "PIN")) {
    if (!a1) {
      sendTo(src, "ERR PIN <sandi baru, 4-12 huruf atau angka>");
      return;
    }
    size_t n = strlen(a1);
    bool ok = n >= 4 && n <= 12;
    for (size_t i = 0; i < n && ok; i++)
      if (!isalnum((unsigned char)a1[i])) ok = false;
    if (!ok) {
      sendTo(src, "ERR sandi harus 4-12 huruf atau angka");
      return;
    }
    strncpy(cfg.pin, a1, sizeof(cfg.pin) - 1);
    cfg.pin[sizeof(cfg.pin) - 1] = 0;
    cfgSave();
    sendTo(src, "OK PIN");
    event("Sandi aplikasi diganti");
  } else if (!strcmp(cmd, "NAME")) {
    if (!a1) {
      sendTo(src, "ERR NAME <nama, maksimal 20 karakter, spasi ditulis _>");
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
    char host[32];
    hostSlug(cfg.name, host, sizeof(host));
    sendf(src, "OK NAME %s (berlaku setelah dinyalakan ulang: http://%s.local)", cfg.name, host);
  } else if (!strcmp(cmd, "WIFI")) {
    wifiCommand(a1, a2, a3, src);
  } else if (!strcmp(cmd, "REBOOT")) {
    if (saveAt) cfgSave();
    sendTo(src, "OK REBOOT");
    rebootAt = millis() + 300;
  } else {
    sendf(src, "ERR perintah tidak dikenal: %s (kirim HELP)", cmd);
  }
}

// ------------------------------------------------------------------------------------ USB input
void pollInputs() {
  static char ub[200];
  static uint8_t un = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      ub[un] = 0;
      if (un) handleLine(ub, TO_USB);
      un = 0;
    } else if (un < sizeof(ub) - 1) ub[un++] = c;
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
  else if (wsCount(true)) on = true;
  else if (wsCount(false)) on = (t / 500) % 2;
  else if (staUp) on = (t % 2000) < 60;
  else if (apOn) on = (t % 2000) < 60 || ((t % 2000) >= 200 && (t % 2000) < 260);
  else on = (t % 1000) < 60;
  digitalWrite(PIN_LED, on ? LOW : HIGH);
}
void telemetry() {
  if (!cfg.telHz || capState == 2) return;
  uint32_t now = millis();
  if ((uint32_t)(now - telT) < 1000u / cfg.telHz) return;
  telT = now;
  uint8_t dest = allDest();
  for (int i = 0; i < WS_N; i++)  // a phone that is behind skips this sample instead of falling further behind
    if ((dest & toWs(i)) && wsUsed(ws[i]) > WS_TX / 3) dest &= ~toWs(i);
  uint8_t flags = (imuOK ? 1 : 0) | (wsCount(true) ? 2 : 0) | (isStill ? 4 : 0) | (satP ? 8 : 0) | (satR ? 16 : 0) |
                  (cal.kind != CAL_IDLE ? 32 : 0) | (accRejPct > 50 ? 64 : 0) | (walkF > 0.5f ? 128 : 0);
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
  Serial.setTxBufferSize(4096);  // CFG/STATUS lines are longer than the 256-byte default and were dropped
  Serial.begin(115200);
  prefs.begin("gimbal", false);
  if (prefs.getBytesLength("accoff") == sizeof(accOff)) prefs.getBytes("accoff", accOff, sizeof(accOff));
  if (prefs.getBytesLength("accgain") == sizeof(accGain)) prefs.getBytes("accgain", accGain, sizeof(accGain));
  for (int k = 0; k < 3; k++) {
    if (!(fabsf(accOff[k]) <= 0.5f)) accOff[k] = 0;
    if (!(accGain[k] >= 0.7f && accGain[k] <= 1.4f)) accGain[k] = 1;
  }
  if (!cfgLoad()) {
    memset(&cfg, 0, sizeof(cfg));
    cfgDefaults(cfg, false);
    cfgSave();
    firstBoot = true;
  } else if (cfgMigrated) {
    cfgSave();
  }
  rstReason = esp_reset_reason();
  bootCount = prefs.getUInt("boots", 0) + 1;
  prefs.putUInt("boots", bootCount);
  brownCount = prefs.getUInt("brown", 0);
  if (rstReason == ESP_RST_BROWNOUT) prefs.putUInt("brown", ++brownCount);
  mode = (Mode)cfg.startMode;

  // servos one after the other, so their start-up current does not add up
  pwmAttach(PIN_SERVO_A);
  pwmAttach(PIN_SERVO_B);
  if (mode != MODE_OFF) {
    pwmWriteUs(pinPitch(), 1500 + cfg.trimP);
    delay(350);
    pwmWriteUs(pinRoll(), 1500 + cfg.trimR);
    delay(350);
  }

  i2cHz = prefs.getUInt("i2chz", 400000);
  if (i2cHz != 100000 && i2cHz != 400000) i2cHz = 400000;
  i2cBegin();
  imuOK = mpuInit();
  if (!imuOK && i2cHz != 100000) {  // long/loose wires: try the slow, robust clock before giving up
    Wire.end();
    i2cHz = 100000;
    i2cBegin();
    imuOK = mpuInit();
    if (imuOK) prefs.putUInt("i2chz", i2cHz);
  }
  wifiInit();
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
  event("Menyala (fw %s). Penyebab mulai ulang: %s. Total mulai ulang karena tegangan turun: %lu", FW_VERSION, resetReasonStr(rstReason),
        (unsigned long)brownCount);
  if (cfgMigrated) event("Pengaturan dari firmware %s tetap dipakai; pengaturan baru diisi nilai bawaan", cfgMigrated);
  if (!imuOK) event("IMU tidak terbaca: periksa VCC->3V3, GND->G, SDA->GPIO6, SCL->GPIO7");
  if (cfg.maxRate > SPEED_WARN)
    event("Peringatan: kecepatan servo %.0f derajat/detik (di atas %.0f): servo cepat panas dan aus.", cfg.maxRate, SPEED_WARN);
  event("I2C IMU %lu kHz", (unsigned long)(i2cHz / 1000));
  if (!cfg.wifiOn) event("WiFi nonaktif (kirim WIFI ON lewat USB untuk mengaktifkan)");
  lastLoopUs = lastTickUs = micros();
  statT = millis();
}

// One 200 Hz control step. Called before and after the network work, so a busy moment on WiFi delays a
// step by less. The schedule is kept on a fixed 5 ms grid (v4.2 restarted the 5 ms from whenever the
// step happened to run, so every late step pushed all later ones back: ~190 Hz with jitter).
bool controlTick() {
  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - (lastLoopUs + LOOP_US)) < 0) return false;
  lastLoopUs += LOOP_US;
  if ((int32_t)(nowUs - lastLoopUs) > (int32_t)(2 * LOOP_US)) lastLoopUs = nowUs;  // far behind: no burst of catch-up steps
  float dt = (uint32_t)(nowUs - lastTickUs) * 1e-6f;  // real time since the last step
  lastTickUs = nowUs;
  if (dt > 0.05f) dt = 0.05f;
  if (dt < 1e-3f) dt = 1e-3f;
  loopCount++;
  uint32_t workT0 = micros();

  imuDt = fminf(imuDt + dt, 0.05f);
  if (mpuRead()) {
    if (!imuOK) {
      imuOK = true;
      attitudeReset();
      event("IMU terbaca lagi");
    }
    imuFailRun = 0;
    for (int k = 0; k < 3; k++) gyrInt[k] += gyrRaw[k] * imuDt;
    estimate(imuDt);
    imuDt = 0;
    if (cfg.autoBias && stillTime > 2.0f && mode == MODE_STAB && cal.kind == CAL_IDLE) {
      for (int k = 0; k < 3; k++) cfg.bias[k] += 0.0005f * gyrRaw[k];  // follow slow gyro drift
    }
  } else {
    imuFailRun++;
    if (imuFailRun == 20 && imuOK) {
      imuOK = false;
      if (cal.kind != CAL_IDLE) calFail("IMU berhenti menjawab");
      event("IMU ERROR: tidak merespons. Servo ditahan. Periksa keempat kabel IMU.");
    }
    if (!imuOK && imuFailRun % 200 == 0) {  // try again every second
      Wire.end();
      if (imuFailRun % 400 == 0) i2cHz = (i2cHz == 400000) ? 100000 : 400000;  // alternate the clock
      i2cBegin();
      if (mpuInit() && prefs.getUInt("i2chz", 400000) != i2cHz) prefs.putUInt("i2chz", i2cHz);
    }
  }
  calUpdate(dt);
  stepUpdate();
  control(dt);
  uint32_t work = micros() - workT0;  // IMU read + estimate + control, the part that must fit in 5 ms
  cpuSum += work;
  if (work > cpuPeak) cpuPeak = work;
  ledUpdate();
  telemetry();

  uint32_t now = millis();
  if ((uint32_t)(now - statT) >= 1000) {
    cpuUs = loopCount ? cpuSum / loopCount : 0;
    cpuMaxUs = cpuPeak;
    cpuSum = cpuPeak = 0;
    loopHz = loopCount;
    loopCount = 0;
    accRejPct = accCount ? 100.0f * accRejCount / accCount : 0;
    accCount = accRejCount = 0;
    statT = now;
    // too many I2C errors at 400 kHz (long or moving wires): drop to 100 kHz for good
    uint32_t errs = i2cErr - i2cErrLast;
    i2cErrLast = i2cErr;
    if (errs >= 10 && i2cHz == 400000) {
      i2cHz = 100000;
      Wire.end();
      i2cBegin();
      mpuInit();
      prefs.putUInt("i2chz", i2cHz);
      event("Banyak galat I2C (%lu/s): IMU dipindah ke 100 kHz", (unsigned long)errs);
    }
  }
  if (saveAt && (int32_t)(now - saveAt) >= 0) cfgSave();
  if (rebootAt && (int32_t)(now - rebootAt) >= 0) ESP.restart();
  return true;
}

void loop() {
  pollInputs();
  controlTick();
  netPoll();
  controlTick();
}
