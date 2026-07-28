/*
  ESP32 ECU Manual V1 - minimal, fully-manual bench firmware.

  Rewritten from scratch (replaces ECU_TestV1_EGT_DRY_START_PATCH) with only two
  automatic subsystems kept, both read-only sensors:
    - RPM measurement on GPIO33 (interrupt + period/window, glitch-filtered)
    - EGT measurement via MAX31855 thermocouple

  EVERYTHING that actuates is manual only - there is NO auto-start sequence, no
  state machine, no checklist interlock, no cooldown, no comm watchdog, no fuel
  closed-loop. Each output is commanded directly over Serial and holds until you
  change it. Nothing turns an output on by itself.

  Interface: Serial only (115200). No WiFi, no web server.
  SD card: used ONLY to save/load the small sensor setup (savecfg/loadcfg). No
  CSV telemetry logging.

  Pins (unchanged from the previous firmware):
    MAX31855: CLK=18 CS=5 DO=19
    RPM in:   33
    ESC pump: 26   ESC starter: 25
    VALVE1:   17   VALVE2: 16    IGN/GLOW: 32
    SD (SPI): CS=13 SCK=14 MOSI=23 MISO=27

  ESC PWM is generated with the raw ESP32 LEDC peripheral (NOT ESP32Servo - that
  library's timer/ISR handling was shown to disturb the RPM interrupt and cap the
  reading around 1500 RPM). ESCs are armed at 900us (below the calibrated 1000us
  MIN) so arming succeeds even with sub-microsecond PWM rounding - see setup().
*/

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif
#include <Adafruit_MAX31855.h>
#include "soc/rtc_cntl_reg.h"  // RTC_CNTL_BROWN_OUT_REG - see setup()

// ---------------- Pins ----------------
#define PIN_EGT_CLK    18
#define PIN_EGT_CS      5
#define PIN_EGT_DO     19
#define PIN_RPM        33
#define PIN_ESC_PUMP   26
#define PIN_ESC_START  25
#define PIN_VALVE_1    17
#define PIN_VALVE_2    16
#define PIN_IGN        32
#define PIN_SD_CS      13
#define PIN_SD_SCK     14
#define PIN_SD_MOSI    23
#define PIN_SD_MISO    27

static const bool IGN_ACTIVE_HIGH   = true;
static const bool VALVE_ACTIVE_HIGH = true;

// Forward-declare the RPM noise enum with a fixed underlying type (legal C++11
// opaque declaration). The Arduino IDE injects its auto-generated function
// prototypes just before the first function definition (escAttach() below), and
// some of those prototypes name RpmNoiseLevel (classifyRpmNoise/rpmNoiseName) -
// without this they would be seen before the real enum definition and fail with
// "'RpmNoiseLevel' does not name a type".
enum RpmNoiseLevel : uint8_t;

// ---------------- ESC PWM (raw LEDC) ----------------
static const int ESC_SAFE_US = 1000;   // idle/zero-throttle after arming
static const int ESC_MIN_US  = 1000;   // calibrated MIN (BLHeliSuite)
static const int ESC_MAX_US  = 2000;   // calibrated MAX
static const int ESC_ARM_US  = 900;    // arm below MIN so arming always succeeds
static const int ESC_PWM_FREQ_HZ  = 50;
static const int ESC_PWM_RES_BITS = 16;
static const int LEDC_CH_PUMP  = 0;    // only used on core <3
static const int LEDC_CH_START = 1;

// Actual LEDC period (us) read back after attach - LEDC can only land on a
// frequency its clock divider allows, so the real period is often a few tens of
// Hz off nominal 50Hz. Duty is computed from this real period, not an assumed
// 20000us, so the pulse width matches the requested us to <1us.
double escPumpPeriodUs  = 20000.0;
double escStartPeriodUs = 20000.0;

double escAttach(uint8_t pin, int legacyChannel) {
  double actualHz;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)legacyChannel;
  ledcAttach(pin, ESC_PWM_FREQ_HZ, ESC_PWM_RES_BITS);
  actualHz = ledcReadFreq(pin);
#else
  ledcSetup(legacyChannel, ESC_PWM_FREQ_HZ, ESC_PWM_RES_BITS);
  ledcAttachPin(pin, legacyChannel);
  actualHz = ledcReadFreq(legacyChannel);
#endif
  return (actualHz > 0.0) ? (1000000.0 / actualHz) : (1000000.0 / ESC_PWM_FREQ_HZ);
}

void escWriteUs(uint8_t pin, int legacyChannel, int us, double periodUs) {
  const uint32_t maxDuty = (1UL << ESC_PWM_RES_BITS) - 1;
  uint32_t duty = (uint32_t)(((double)us * (double)maxDuty) / periodUs + 0.5);
  if (duty > maxDuty) duty = maxDuty;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(legacyChannel, duty);
#endif
}

// ---------------- RPM measurement ----------------
static const uint32_t RPM_SAMPLE_MS         = 100;
static const uint32_t RPM_SIGNAL_TIMEOUT_MS = 1000;

// Software glitch filter (min quiet-period between accepted edges).
volatile uint32_t rpmMinPulseUs = 120;
int rpmEdgeMode = RISING;

void IRAM_ATTR rpmISR();

volatile uint32_t isrLastRawEdgeUs = 0;
volatile uint32_t isrLastAcceptedPulseUs = 0;
volatile uint32_t isrLastPeriodUs = 0;
volatile uint32_t isrRawEdges = 0;
volatile uint32_t isrAcceptedPulses = 0;
volatile uint32_t isrRejectedEdges = 0;
volatile uint32_t isrAcceptedIntervals = 0;
volatile uint32_t isrMinDtUs = 0xFFFFFFFFUL;
volatile uint32_t isrMaxDtUs = 0;
volatile uint64_t isrSumDtUs = 0;
volatile uint64_t isrSumDtSqUs = 0;

enum RpmNoiseLevel : uint8_t { RPM_CLEAN, RPM_WARN, RPM_NOISY, RPM_REST_NOISE, RPM_NO_SIGNAL };

struct RpmState {
  float rpm = 0.0f, rpmWindow = 0.0f, rpmPeriod = 0.0f;
  float avgIntervalUs = 0.0f, jitterPct = 0.0f, rejectPct = 0.0f, rpmDiffPct = 0.0f;
  bool signalRecent = false, restGuardActive = false, restPulseNoise = false;
  uint32_t acceptedWindow = 0, rawEdges = 0, rejectedEdges = 0, validIntervals = 0;
  uint32_t lastPeriodUs = 0, minIntervalUs = 0, maxIntervalUs = 0, filterUs = 0;
  uint32_t lastComputedMs = 0, lastComputedUs = 0;
  int pinLevel = 0;
  RpmNoiseLevel noise = RPM_NO_SIGNAL;
} rpmData;

uint8_t  pulsesPerRev = 1;
bool     rpmDetailMode = false;
// When true the rest-guard is disabled so the sensor can be hand-spun for
// calibration and still show live RPM. OFF by default: with outputs off, ambient
// EMI on GPIO33 can look like a steady low RPM, so the guard normally forces 0.
bool     rpmCalMode = false;

const char* rpmNoiseName(RpmNoiseLevel n) {
  switch (n) {
    case RPM_CLEAN: return "CLEAN";
    case RPM_WARN: return "WARN";
    case RPM_NOISY: return "NOISY";
    case RPM_REST_NOISE: return "REST_NOISE";
    case RPM_NO_SIGNAL: return "NO_SIGNAL";
    default: return "UNKNOWN";
  }
}
const char* rpmEdgeName() { return (rpmEdgeMode == FALLING) ? "FALLING" : "RISING"; }

// ---------------- Output state ----------------
int pumpUs = ESC_SAFE_US, startUs = ESC_SAFE_US;
bool ignCmd = false, valve1Cmd = false, valve2Cmd = false;

bool allOutputsOff() {
  return startUs <= ESC_SAFE_US && pumpUs <= ESC_SAFE_US && !ignCmd && !valve1Cmd && !valve2Cmd;
}

bool rpmAtRestGuardCondition() {
  // All outputs off + not calibrating => any edge is treated as at-rest noise.
  if (rpmCalMode) return false;
  return allOutputsOff();
}

RpmNoiseLevel classifyRpmNoise(bool recent, uint32_t raw, uint32_t accepted, uint32_t rejected,
                               float rejectPct, float jitterPct, float rpmDiffPct, uint32_t intervals) {
  if (!recent && raw == 0) return RPM_NO_SIGNAL;
  if (raw > 1 && accepted == 0) return RPM_NOISY;
  if (rejected >= 3 || rejectPct > 20.0f ||
      (intervals >= 5 && jitterPct > 30.0f) || (rpmDiffPct > 30.0f)) return RPM_NOISY;
  if (rejected > 0 || rejectPct > 5.0f ||
      (intervals >= 3 && jitterPct > 15.0f) || (rpmDiffPct > 15.0f)) return RPM_WARN;
  return RPM_CLEAN;
}

void attachRpmInterrupt() {
  detachInterrupt(PIN_RPM);
  attachInterrupt(digitalPinToInterrupt(PIN_RPM), rpmISR, rpmEdgeMode);
}

void IRAM_ATTR rpmISR() {
  uint32_t nowUs = micros();
  if (nowUs == 0) nowUs = 1;
  isrRawEdges++;
  if (isrLastRawEdgeUs == 0) { isrLastRawEdgeUs = nowUs; return; }
  uint32_t dtRawUs = nowUs - isrLastRawEdgeUs;
  isrLastRawEdgeUs = nowUs;
  uint32_t filterUs = rpmMinPulseUs;
  if (dtRawUs < filterUs) { isrRejectedEdges++; return; }
  if (isrLastAcceptedPulseUs != 0) {
    uint32_t dtAcceptedUs = nowUs - isrLastAcceptedPulseUs;
    // Adaptive mask: reject edges within half of the last accepted period (a
    // turbine can't double speed in one rev), catching isolated EMI spikes.
    uint32_t maskUs = filterUs;
    if (isrLastPeriodUs > 0 && (isrLastPeriodUs >> 1) > maskUs) maskUs = (isrLastPeriodUs >> 1);
    if (dtAcceptedUs < maskUs) { isrRejectedEdges++; return; }
    isrLastPeriodUs = dtAcceptedUs;
    isrAcceptedIntervals++;
    isrSumDtUs += dtAcceptedUs;
    isrSumDtSqUs += (uint64_t)dtAcceptedUs * (uint64_t)dtAcceptedUs;
    if (dtAcceptedUs < isrMinDtUs) isrMinDtUs = dtAcceptedUs;
    if (dtAcceptedUs > isrMaxDtUs) isrMaxDtUs = dtAcceptedUs;
  }
  isrLastAcceptedPulseUs = nowUs;
  isrAcceptedPulses++;
}

void resetRpmStats() {
  noInterrupts();
  isrLastRawEdgeUs = 0; isrLastAcceptedPulseUs = 0; isrLastPeriodUs = 0;
  isrRawEdges = 0; isrAcceptedPulses = 0; isrRejectedEdges = 0; isrAcceptedIntervals = 0;
  isrSumDtUs = 0; isrSumDtSqUs = 0; isrMinDtUs = 0xFFFFFFFFUL; isrMaxDtUs = 0;
  interrupts();
  rpmData = RpmState();
  Serial.println("RPM stats reset.");
}

void updateRpm() {
  uint32_t nowMs = millis();
  if (nowMs - rpmData.lastComputedMs < RPM_SAMPLE_MS) return;

  uint32_t accepted, raw, rejected, validN, minDt, maxDt, lastPulseUs, lastPeriodUs, filterUs;
  uint64_t sumDt, sumDtSq;
  noInterrupts();
  accepted = isrAcceptedPulses; raw = isrRawEdges; rejected = isrRejectedEdges;
  validN = isrAcceptedIntervals; sumDt = isrSumDtUs; sumDtSq = isrSumDtSqUs;
  minDt = isrMinDtUs; maxDt = isrMaxDtUs; lastPulseUs = isrLastAcceptedPulseUs;
  lastPeriodUs = isrLastPeriodUs; filterUs = rpmMinPulseUs;
  isrAcceptedPulses = 0; isrRawEdges = 0; isrRejectedEdges = 0; isrAcceptedIntervals = 0;
  isrSumDtUs = 0; isrSumDtSqUs = 0; isrMinDtUs = 0xFFFFFFFFUL; isrMaxDtUs = 0;
  interrupts();

  uint32_t nowUs = micros();
  if (nowUs == 0) nowUs = 1;
  uint32_t windowUs = (rpmData.lastComputedUs == 0) ? (RPM_SAMPLE_MS * 1000UL) : (nowUs - rpmData.lastComputedUs);
  rpmData.lastComputedUs = nowUs;
  rpmData.lastComputedMs = nowMs;

  rpmData.acceptedWindow = accepted; rpmData.rawEdges = raw; rpmData.rejectedEdges = rejected;
  rpmData.validIntervals = validN; rpmData.lastPeriodUs = lastPeriodUs;
  rpmData.minIntervalUs = (minDt == 0xFFFFFFFFUL) ? 0 : minDt;
  rpmData.maxIntervalUs = maxDt; rpmData.filterUs = filterUs;
  rpmData.pinLevel = digitalRead(PIN_RPM);

  rpmData.restGuardActive = rpmAtRestGuardCondition();
  rpmData.signalRecent = lastPulseUs != 0 && ((uint32_t)(nowUs - lastPulseUs) <= RPM_SIGNAL_TIMEOUT_MS * 1000UL);
  rpmData.restPulseNoise = rpmData.restGuardActive && (raw > 0 || accepted > 0 || rpmData.signalRecent);

  rpmData.rejectPct = (raw > 0) ? ((float)rejected * 100.0f / (float)raw) : 0.0f;
  rpmData.rpmWindow = (accepted > 0 && windowUs > 0) ?
                      ((float)accepted * 60000000.0f / ((float)windowUs * (float)pulsesPerRev)) : 0.0f;
  rpmData.rpmPeriod = 0.0f;
  if (rpmData.signalRecent && lastPeriodUs > 0)
    rpmData.rpmPeriod = 60000000.0f / ((float)lastPeriodUs * (float)pulsesPerRev);

  if (validN > 0) {
    rpmData.avgIntervalUs = (float)sumDt / (float)validN;
    double meanUs = (double)sumDt / (double)validN;
    double meanSqUs = (double)sumDtSq / (double)validN;
    double variance = meanSqUs - meanUs * meanUs;
    if (variance < 0.0) variance = 0.0;
    float stddevUs = sqrtf((float)variance);
    rpmData.jitterPct = (rpmData.avgIntervalUs > 0.0f) ? (stddevUs * 100.0f / rpmData.avgIntervalUs) : 0.0f;
  } else if (!rpmData.signalRecent) {
    rpmData.avgIntervalUs = 0.0f; rpmData.jitterPct = 0.0f;
  }

  rpmData.rpmDiffPct = 0.0f;
  if (rpmData.rpmWindow > 0.0f && rpmData.rpmPeriod > 0.0f) {
    float base = max(rpmData.rpmWindow, rpmData.rpmPeriod);
    rpmData.rpmDiffPct = fabsf(rpmData.rpmWindow - rpmData.rpmPeriod) * 100.0f / base;
  }

  if (rpmData.restPulseNoise) {
    rpmData.rpm = 0.0f; rpmData.signalRecent = false; rpmData.noise = RPM_REST_NOISE;
  } else {
    if (rpmData.signalRecent && rpmData.rpmPeriod > 0.0f) rpmData.rpm = rpmData.rpmPeriod;
    else if (accepted > 0) rpmData.rpm = rpmData.rpmWindow;
    else rpmData.rpm = 0.0f;
    rpmData.noise = classifyRpmNoise(rpmData.signalRecent, raw, accepted, rejected,
                                     rpmData.rejectPct, rpmData.jitterPct, rpmData.rpmDiffPct, validN);
  }
}

// ---------------- EGT (MAX31855) ----------------
static const uint32_t EGT_READ_PERIOD_MS = 120;
Adafruit_MAX31855 thermo(PIN_EGT_CLK, PIN_EGT_CS, PIN_EGT_DO);

struct EgtState {
  bool ok = false;
  float c = NAN, prevC = NAN, gradientCps = 0.0f;
  uint8_t fault = 0;
  uint32_t lastReadMs = 0, lastGoodMs = 0;
} egt;

String egtFaultString(uint8_t f) {
  if (f == 0) return "SPI/WIRING";
  String s = "";
  if (f & MAX31855_FAULT_OPEN) s += "OPEN ";
  if (f & MAX31855_FAULT_SHORT_GND) s += "SHORT_GND ";
  if (f & MAX31855_FAULT_SHORT_VCC) s += "SHORT_VCC ";
  s.trim(); return s;
}

void updateEgt() {
  uint32_t nowMs = millis();
  if (nowMs - egt.lastReadMs < EGT_READ_PERIOD_MS) return;
  egt.lastReadMs = nowMs;
  uint8_t fault = thermo.readError();
  double tc = thermo.readCelsius();
  if (isnan(tc) || fault != 0) {
    egt.ok = false; egt.fault = fault; egt.gradientCps = 0; return;
  }
  if (egt.ok && !isnan(egt.c)) {
    float dtS = (float)(nowMs - egt.lastGoodMs) / 1000.0f;
    if (dtS > 0.05f) egt.gradientCps = ((float)tc - egt.c) / dtS;
  } else egt.gradientCps = 0;
  egt.prevC = egt.c; egt.c = (float)tc; egt.ok = true; egt.fault = 0; egt.lastGoodMs = nowMs;
}

// ---------------- SD config (setup only, no telemetry) ----------------
SPIClass sdSPI(VSPI);
bool sdOk = false, sdMounted = false;
static const uint32_t SD_SPI_HZ = 1000000;
static const char* CONFIG_PATH = "/ECUCFG.TXT";

bool mountSd() {
  if (sdMounted) return sdOk;
  sdMounted = true;
  sdSPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (!SD.begin(PIN_SD_CS, sdSPI, SD_SPI_HZ)) { sdOk = false; Serial.println("SD: begin() FAIL."); return false; }
  if (SD.cardType() == CARD_NONE) { sdOk = false; Serial.println("SD: no card."); return false; }
  sdOk = true; return true;
}

static int clampInt(long v, long lo, long hi) { return (int)(v < lo ? lo : (v > hi ? hi : v)); }

bool saveConfigToSd() {
  if (!mountSd()) { Serial.println("SAVECFG: no SD."); return false; }
  SD.remove(CONFIG_PATH);
  File f = SD.open(CONFIG_PATH, FILE_WRITE);
  if (!f) { Serial.println("SAVECFG: open FAIL."); return false; }
  f.println("# ECU Manual V1 sensor setup - auto-loaded on boot");
  f.print("ppr=");       f.println((int)pulsesPerRev);
  f.print("rpmfilter="); f.println((uint32_t)rpmMinPulseUs);
  f.print("rpmedge=");   f.println(rpmEdgeName());
  f.close();
  Serial.println("SAVECFG: OK -> /ECUCFG.TXT");
  return true;
}

bool loadConfigFromSd() {
  if (!mountSd()) return false;
  if (!SD.exists(CONFIG_PATH)) { Serial.println("LOADCFG: no /ECUCFG.TXT, using defaults."); return false; }
  File f = SD.open(CONFIG_PATH, FILE_READ);
  if (!f) { Serial.println("LOADCFG: open FAIL."); return false; }
  int applied = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n'); line.trim();
    if (line.length() == 0 || line[0] == '#') continue;
    int eq = line.indexOf('='); if (eq <= 0) continue;
    String key = line.substring(0, eq); key.trim();
    String val = line.substring(eq + 1); val.trim();
    long n = val.toInt();
    if      (key == "ppr")       pulsesPerRev = (n == 2) ? 2 : 1;
    else if (key == "rpmfilter") rpmMinPulseUs = (uint32_t)clampInt(n, 20, 5000);
    else if (key == "rpmedge")   rpmEdgeMode = (val == "FALLING") ? FALLING : RISING;
    else continue;
    applied++;
  }
  f.close();
  Serial.print("LOADCFG: applied "); Serial.print(applied); Serial.println(" keys.");
  return applied > 0;
}

// ---------------- Output apply ----------------
void applyOutputs() {
  pumpUs = constrain(pumpUs, ESC_MIN_US, ESC_MAX_US);
  startUs = constrain(startUs, ESC_MIN_US, ESC_MAX_US);
  static int lastPumpUs = -1, lastStartUs = -1;
  if (pumpUs != lastPumpUs) { escWriteUs(PIN_ESC_PUMP, LEDC_CH_PUMP, pumpUs, escPumpPeriodUs); lastPumpUs = pumpUs; }
  if (startUs != lastStartUs) { escWriteUs(PIN_ESC_START, LEDC_CH_START, startUs, escStartPeriodUs); lastStartUs = startUs; }
  digitalWrite(PIN_IGN, (ignCmd == IGN_ACTIVE_HIGH) ? HIGH : LOW);
  digitalWrite(PIN_VALVE_1, (valve1Cmd == VALVE_ACTIVE_HIGH) ? HIGH : LOW);
  digitalWrite(PIN_VALVE_2, (valve2Cmd == VALVE_ACTIVE_HIGH) ? HIGH : LOW);
}

void allOff() {
  pumpUs = ESC_SAFE_US; startUs = ESC_SAFE_US;
  ignCmd = false; valve1Cmd = false; valve2Cmd = false;
  applyOutputs();
}

// ---------------- ESC throttle-range calibration ----------------
bool escCalActive = false;
uint32_t escCalPhaseUntilMs = 0;
static const uint32_t ESC_CAL_MAX_HOLD_MS = 5000;

// ---------------- Serial console ----------------
long numberAfter(const String& cmd, const String& prefix) { return cmd.substring(prefix.length()).toInt(); }

void printHelp() {
  Serial.println("===== ECU Manual V1 - commands =====");
  Serial.println("help | status | showcfg");
  Serial.println("rpmdetail | rpmdetail on/off | rpmreset");
  Serial.println("set rpmfilter <20..5000>   -> RPM glitch filter (us)");
  Serial.println("set rpmedge rising|falling -> RPM interrupt edge");
  Serial.println("set ppr 1|2                -> pulses per revolution");
  Serial.println("set rpmcal on/off          -> disable/restore rest-guard for hand-spin calibration");
  Serial.println("starter <1000..2000> | starter off  -> hold starter ESC PWM");
  Serial.println("pump <1000..2000>    | pump off     -> hold pump ESC PWM");
  Serial.println("ign on | ign off            -> igniter/glow");
  Serial.println("valve1 on|off | valve2 on|off");
  Serial.println("esccal start | esccal cancel -> ESC throttle-range calib: PUMP+STARTER to 2000us, auto-drop to 1000us after 5s");
  Serial.println("alloff | stop               -> all outputs to safe");
  Serial.println("savecfg | loadcfg           -> save/reload sensor setup to SD");
  Serial.println("(Fully manual: every output holds until you change it. Nothing auto-starts.)");
}

void printStatus() {
  Serial.print("EGT=");
  if (egt.ok) { Serial.print(egt.c, 1); Serial.print("C"); } else { Serial.print("ERR("); Serial.print(egtFaultString(egt.fault)); Serial.print(")"); }
  Serial.print(" | dEGT="); Serial.print(egt.gradientCps, 1);
  Serial.print(" | RPM="); Serial.print(rpmData.rpm, 0);
  Serial.print(" | SIG="); Serial.print(rpmData.signalRecent ? "OK" : (rpmData.noise == RPM_REST_NOISE ? "REST" : "LOST"));
  Serial.print(" | RNOISE="); Serial.print(rpmNoiseName(rpmData.noise));
  Serial.print(" | PUMP="); Serial.print(pumpUs); Serial.print("us");
  Serial.print(" | START="); Serial.print(startUs); Serial.print("us");
  Serial.print(" | IGN="); Serial.print(ignCmd ? 1 : 0);
  Serial.print(" | V1="); Serial.print(valve1Cmd ? 1 : 0);
  Serial.print(" | V2="); Serial.print(valve2Cmd ? 1 : 0);
  Serial.print(" | SD="); Serial.print(sdOk ? "OK" : "-");
  if (rpmCalMode) Serial.print(" | RPMCAL");
  Serial.println();
}

void printRpmDetail() {
  Serial.print("RPM_DETAIL= RPM="); Serial.print(rpmData.rpm, 0);
  Serial.print(" RPMw="); Serial.print(rpmData.rpmWindow, 0);
  Serial.print(" RPMp="); Serial.print(rpmData.rpmPeriod, 0);
  Serial.print(" | pin="); Serial.print(rpmData.pinLevel);
  Serial.print(" acc="); Serial.print(rpmData.acceptedWindow);
  Serial.print(" raw="); Serial.print(rpmData.rawEdges);
  Serial.print(" rej="); Serial.print(rpmData.rejectedEdges);
  Serial.print(" rej%="); Serial.print(rpmData.rejectPct, 1);
  Serial.print(" | per="); Serial.print(rpmData.lastPeriodUs); Serial.print("us");
  Serial.print(" min="); Serial.print(rpmData.minIntervalUs); Serial.print("us");
  Serial.print(" max="); Serial.print(rpmData.maxIntervalUs); Serial.print("us");
  Serial.print(" avg="); Serial.print(rpmData.avgIntervalUs, 1); Serial.print("us");
  Serial.print(" | JIT="); Serial.print(rpmData.jitterPct, 1); Serial.print("%");
  Serial.print(" DIFF="); Serial.print(rpmData.rpmDiffPct, 1); Serial.print("%");
  Serial.print(" | filt="); Serial.print(rpmData.filterUs); Serial.print("us");
  Serial.print(" edge="); Serial.print(rpmEdgeName());
  Serial.print(" RNOISE="); Serial.print(rpmNoiseName(rpmData.noise));
  Serial.println();
}

void showCfg() {
  Serial.println("===== SETUP =====");
  Serial.print("ppr="); Serial.println((int)pulsesPerRev);
  Serial.print("rpmfilter="); Serial.print((uint32_t)rpmMinPulseUs); Serial.println(" us");
  Serial.print("rpmedge="); Serial.println(rpmEdgeName());
  Serial.print("ESC period pump/start="); Serial.print(escPumpPeriodUs, 1); Serial.print("/"); Serial.print(escStartPeriodUs, 1); Serial.println(" us");
  Serial.print("SD="); Serial.println(sdOk ? "OK" : "FAIL/NONE");
  Serial.println("=================");
}

void handleCommand(String cmd) {
  cmd.trim(); cmd.toLowerCase();
  if (!cmd.length()) return;

  if (cmd == "help") { printHelp(); return; }
  if (cmd == "status") { printStatus(); return; }
  if (cmd == "showcfg" || cmd == "cfg") { showCfg(); return; }
  if (cmd == "rpmdetail") { updateRpm(); printRpmDetail(); return; }
  if (cmd == "rpmdetail on") { rpmDetailMode = true; resetRpmStats(); Serial.println("RPM detail ON."); return; }
  if (cmd == "rpmdetail off") { rpmDetailMode = false; Serial.println("RPM detail OFF."); return; }
  if (cmd == "rpmreset") { resetRpmStats(); return; }

  if (cmd.startsWith("set rpmfilter ")) {
    int f = numberAfter(cmd, "set rpmfilter ");
    if (f < 20 || f > 5000) { Serial.println("ERROR: rpmfilter 20..5000 us"); return; }
    noInterrupts(); rpmMinPulseUs = (uint32_t)f; interrupts();
    resetRpmStats(); Serial.print("rpmFilterUs="); Serial.println(f); return;
  }
  if (cmd.startsWith("set rpmedge ")) {
    String e = cmd.substring(String("set rpmedge ").length()); e.trim();
    if (e == "rising") rpmEdgeMode = RISING;
    else if (e == "falling") rpmEdgeMode = FALLING;
    else { Serial.println("ERROR: set rpmedge rising|falling"); return; }
    resetRpmStats(); attachRpmInterrupt();
    Serial.print("rpmEdge="); Serial.println(rpmEdgeName()); return;
  }
  if (cmd.startsWith("set ppr ")) {
    int p = numberAfter(cmd, "set ppr ");
    if (p != 1 && p != 2) { Serial.println("ERROR: ppr 1 or 2"); return; }
    pulsesPerRev = p; resetRpmStats(); Serial.println("OK"); return;
  }
  if (cmd == "set rpmcal on") { rpmCalMode = true; resetRpmStats(); Serial.println("RPM CAL ON: rest-guard disabled (hand-spin). 'set rpmcal off' when done."); return; }
  if (cmd == "set rpmcal off") { rpmCalMode = false; resetRpmStats(); Serial.println("RPM CAL OFF: rest-guard restored."); return; }

  if (cmd.startsWith("starter ")) {
    String arg = cmd.substring(String("starter ").length()); arg.trim();
    if (arg == "off") { startUs = ESC_SAFE_US; applyOutputs(); Serial.println("STARTER OFF."); return; }
    int us = arg.toInt();
    if (us < ESC_MIN_US || us > ESC_MAX_US) { Serial.println("ERROR: starter 1000..2000 (or 'starter off')"); return; }
    startUs = us; applyOutputs();
    Serial.print("STARTER HOLD at "); Serial.print(us); Serial.println("us"); return;
  }
  if (cmd.startsWith("pump ")) {
    String arg = cmd.substring(String("pump ").length()); arg.trim();
    if (arg == "off") { pumpUs = ESC_SAFE_US; applyOutputs(); Serial.println("PUMP OFF."); return; }
    int us = arg.toInt();
    if (us < ESC_MIN_US || us > ESC_MAX_US) { Serial.println("ERROR: pump 1000..2000 (or 'pump off')"); return; }
    pumpUs = us; applyOutputs();
    Serial.print("PUMP HOLD at "); Serial.print(us); Serial.println("us"); return;
  }
  if (cmd == "ign on")  { ignCmd = true;  applyOutputs(); Serial.println("IGN ON.");  return; }
  if (cmd == "ign off") { ignCmd = false; applyOutputs(); Serial.println("IGN OFF."); return; }
  if (cmd == "valve1 on")  { valve1Cmd = true;  applyOutputs(); Serial.println("VALVE1 ON.");  return; }
  if (cmd == "valve1 off") { valve1Cmd = false; applyOutputs(); Serial.println("VALVE1 OFF."); return; }
  if (cmd == "valve2 on")  { valve2Cmd = true;  applyOutputs(); Serial.println("VALVE2 ON.");  return; }
  if (cmd == "valve2 off") { valve2Cmd = false; applyOutputs(); Serial.println("VALVE2 OFF."); return; }

  if (cmd == "esccal start") {
    escCalActive = true;
    startUs = ESC_MAX_US; pumpUs = ESC_MAX_US; applyOutputs();
    escCalPhaseUntilMs = millis() + ESC_CAL_MAX_HOLD_MS;
    Serial.print("ESC CAL: MAX 2000us on PUMP+STARTER. Power the ESCs now. Auto-drop to MIN in ");
    Serial.print(ESC_CAL_MAX_HOLD_MS / 1000); Serial.println("s."); return;
  }
  if (cmd == "esccal cancel" || cmd == "esccal off") {
    escCalActive = false; escCalPhaseUntilMs = 0;
    startUs = ESC_SAFE_US; pumpUs = ESC_SAFE_US; applyOutputs();
    Serial.println("ESC CAL cancelled, outputs safe."); return;
  }

  if (cmd == "alloff" || cmd == "stop" || cmd == "off") { allOff(); Serial.println("ALL OUTPUTS SAFE."); return; }
  if (cmd == "savecfg") { saveConfigToSd(); return; }
  if (cmd == "loadcfg") { if (loadConfigFromSd()) { resetRpmStats(); attachRpmInterrupt(); Serial.println("Config reloaded."); } else Serial.println("No saved config applied."); return; }

  Serial.println("Unknown command. Type help");
}

// ---------------- setup / loop ----------------
static const uint32_t STATUS_PRINT_MS = 250;
uint32_t lastStatusPrintMs = 0;
String serialCmdBuf = "";

void setup() {
  // Diagnostic-only: the previous firmware showed repeated brownout resets during
  // the ESC arm window on this board's supply. Disabling the detector lets the
  // chip run through a brief under-voltage dip instead of reset-looping. This does
  // NOT fix a real supply problem - add a bulk cap (470-1000uF low-ESR + 100nF) at
  // the ESP32 power input and/or separate it from the ESC/motor rail, then this
  // line can be removed.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(400);

  pinMode(PIN_IGN, OUTPUT); pinMode(PIN_VALVE_1, OUTPUT); pinMode(PIN_VALVE_2, OUTPUT);
  pinMode(PIN_RPM, INPUT);
  digitalWrite(PIN_IGN, IGN_ACTIVE_HIGH ? LOW : HIGH);
  digitalWrite(PIN_VALVE_1, VALVE_ACTIVE_HIGH ? LOW : HIGH);
  digitalWrite(PIN_VALVE_2, VALVE_ACTIVE_HIGH ? LOW : HIGH);

  loadConfigFromSd();
  attachRpmInterrupt();

  escPumpPeriodUs  = escAttach(PIN_ESC_PUMP, LEDC_CH_PUMP);
  escStartPeriodUs = escAttach(PIN_ESC_START, LEDC_CH_START);
  Serial.print("ESC PWM actual period: pump="); Serial.print(escPumpPeriodUs, 2);
  Serial.print("us start="); Serial.print(escStartPeriodUs, 2); Serial.println("us (nominal 20000us)");

  // Arm ESCs at 900us (below MIN) so arming always succeeds despite PWM rounding,
  // then hold before switching to the normal safe 1000us.
  Serial.print("ESC ARM: "); Serial.print(ESC_ARM_US); Serial.println("us, holding 4s - listen for the arm beep...");
  escWriteUs(PIN_ESC_PUMP, LEDC_CH_PUMP, ESC_ARM_US, escPumpPeriodUs);
  escWriteUs(PIN_ESC_START, LEDC_CH_START, ESC_ARM_US, escStartPeriodUs);
  delay(4000);
  allOff();

  bool thermoOk = thermo.begin();
  thermo.setFaultChecks(MAX31855_FAULT_ALL);

  Serial.println("ECU Manual V1 booted (Serial-only, fully manual).");
  Serial.print("MAX31855 begin() = "); Serial.println(thermoOk ? "OK" : "CHECK_WIRING");
  Serial.print("RPM edge="); Serial.print(rpmEdgeName());
  Serial.print(" filter="); Serial.print((uint32_t)rpmMinPulseUs);
  Serial.print("us ppr="); Serial.println((int)pulsesPerRev);
  Serial.println("Type help for commands. All outputs safe.");
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (serialCmdBuf.length()) { handleCommand(serialCmdBuf); serialCmdBuf = ""; }
    } else if (c != '\r') {
      if (serialCmdBuf.length() < 80) serialCmdBuf += c;
    }
  }

  // ESC throttle-range calibration: drop MAX->MIN after the hold window (non-blocking).
  if (escCalActive && escCalPhaseUntilMs > 0 && millis() >= escCalPhaseUntilMs) {
    startUs = ESC_MIN_US; pumpUs = ESC_MIN_US; applyOutputs();
    escCalPhaseUntilMs = 0; escCalActive = false;
    Serial.println("ESC CAL: dropped to MIN 1000us (done).");
  }

  updateRpm();
  updateEgt();
  applyOutputs();

  if (millis() - lastStatusPrintMs >= STATUS_PRINT_MS) {
    lastStatusPrintMs = millis();
    printStatus();
    if (rpmDetailMode) printRpmDetail();
  }
}
