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
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"

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

enum RpmNoiseLevel : uint8_t { RPM_CLEAN, RPM_WARN, RPM_NOISY, RPM_REST_NOISE, RPM_NO_SIGNAL };
static const bool IGN_ACTIVE_HIGH   = true;
static const bool VALVE_ACTIVE_HIGH = true;

// ---------------- ESC PWM (raw LEDC) ----------------
static const int ESC_SAFE_US = 900;   
static const int ESC_MIN_US  = 900;   
static const int ESC_MAX_US  = 2000;   
static const int ESC_ARM_US  = 900;    
static const int ESC_PWM_FREQ_HZ  = 50;
static const int ESC_PWM_RES_BITS = 16;
static const int LEDC_CH_PUMP  = 0;    
static const int LEDC_CH_START = 1;

double escPumpPeriodUs  = 20000.0;
double escStartPeriodUs = 20000.0;

// Forward declarations
void handleCommand(String cmd);
void IRAM_ATTR rpmISR();
void attachRpmInterrupt();

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
  double dutyRatio = (double)us / periodUs;
  uint32_t duty = (uint32_t)(dutyRatio * (double)maxDuty + 0.5);
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

volatile uint32_t rpmMinPulseUs = 120;
int rpmEdgeMode = RISING;

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

String serialCmdBuf = "";

void processSerialRx() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if ((c < 32 || c > 126) && c != '\n' && c != '\r') {
      continue; 
    }

    if (c == '\n') {
      if (serialCmdBuf.length() > 0) {
        Serial.print("-> [RX Received]: ");
        Serial.println(serialCmdBuf);

        handleCommand(serialCmdBuf);
        serialCmdBuf = "";
      }
    } else if (c != '\r') {
      if (serialCmdBuf.length() < 80) {
        serialCmdBuf += c;
      }
    }
  }
}

void attachRpmInterrupt() {
  detachInterrupt(digitalPinToInterrupt(PIN_RPM));
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

// ---------------- SD config (Chuyển sang HSPI để không đụng VSPI của MAX31855) ----------------
SPIClass sdSPI(HSPI);
bool sdOk = false, sdMounted = false;
static const uint32_t SD_SPI_HZ = 1000000;
static const char* CONFIG_PATH = "/ECUCFG.TXT";

bool mountSd() {
  if (sdMounted) return sdOk;
  sdSPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (!SD.begin(PIN_SD_CS, sdSPI, SD_SPI_HZ)) { sdOk = false; sdMounted = true; Serial.println("SD: begin() FAIL."); return false; }
  if (SD.cardType() == CARD_NONE) { sdOk = false; sdMounted = true; Serial.println("SD: no card."); return false; }
  sdOk = true; sdMounted = true; return true;
}

static int clampInt(long v, long lo, long hi) { return (int)(v < lo ? lo : (v > hi ? hi : v)); }

bool saveConfigToSd() {
  if (!mountSd()) { Serial.println("SAVECFG: no SD."); return false; }
  SD.remove(CONFIG_PATH);
  File f = SD.open(CONFIG_PATH, FILE_WRITE);
  if (!f) { Serial.println("SAVECFG: open FAIL."); return false; }
  f.println("# ECU Manual V1 sensor setup - auto-loaded on boot");
  f.print("ppr=");        f.println((int)pulsesPerRev);
  f.print("rpmfilter="); f.println((uint32_t)rpmMinPulseUs);
  f.print("rpmedge=");   f.println(rpmEdgeName());
  
  // LƯU THÊM GIÁ TRỊ PUMP VÀ STARTER
  f.print("pump=");      f.println(pumpUs);
  f.print("start=");     f.println(startUs);
  
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
    if      (key == "ppr")        pulsesPerRev = (n == 2) ? 2 : 1;
    else if (key == "rpmfilter") {
      noInterrupts();
      rpmMinPulseUs = (uint32_t)clampInt(n, 20, 5000);
      interrupts();
    }
    else if (key == "rpmedge")   rpmEdgeMode = (val == "FALLING") ? FALLING : RISING;
    
    // TẢI THÊM GIÁ TRỊ PUMP VÀ STARTER TỪ FILE
    else if (key == "pump")      pumpUs = clampInt(n, ESC_MIN_US, ESC_MAX_US);
    else if (key == "start")     startUs = clampInt(n, ESC_MIN_US, ESC_MAX_US);
    
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
  Serial.println("esccal start | esccal cancel -> ESC throttle-range calib");
  Serial.println("alloff | stop               -> all outputs to safe");
  Serial.println("savecfg | loadcfg           -> save/reload sensor setup to SD");
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
  if (cmd == "set rpmcal on") { rpmCalMode = true; resetRpmStats(); Serial.println("RPM CAL ON."); return; }
  if (cmd == "set rpmcal off") { rpmCalMode = false; resetRpmStats(); Serial.println("RPM CAL OFF."); return; }

  if (cmd.startsWith("starter ")) {
    String arg = cmd.substring(String("starter ").length()); arg.trim();
    if (arg == "off") { startUs = ESC_SAFE_US; applyOutputs(); Serial.println("STARTER OFF."); return; }
    int us = arg.toInt();
    if (us < ESC_MIN_US || us > ESC_MAX_US) { Serial.println("ERROR: starter 1000..2000"); return; }
    startUs = us; applyOutputs();
    Serial.print("STARTER HOLD at "); Serial.print(us); Serial.println("us"); return;
  }
  if (cmd.startsWith("pump ")) {
    String arg = cmd.substring(String("pump ").length()); arg.trim();
    if (arg == "off") { pumpUs = ESC_SAFE_US; applyOutputs(); Serial.println("PUMP OFF."); return; }
    int us = arg.toInt();
    if (us < ESC_MIN_US || us > ESC_MAX_US) { Serial.println("ERROR: pump 1000..2000"); return; }
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
    Serial.println("ESC CAL: MAX 2000us. Auto-drop to MIN after 5s."); return;
  }
  if (cmd == "esccal cancel" || cmd == "esccal off") {
    escCalActive = false; escCalPhaseUntilMs = 0;
    startUs = ESC_SAFE_US; pumpUs = ESC_SAFE_US; applyOutputs();
    Serial.println("ESC CAL cancelled."); return;
  }

  if (cmd == "alloff" || cmd == "stop" || cmd == "off") { allOff(); Serial.println("ALL OUTPUTS SAFE."); return; }
  if (cmd == "savecfg") { saveConfigToSd(); return; }
  if (cmd == "loadcfg") { 
    if (loadConfigFromSd()) { 
      resetRpmStats(); 
      attachRpmInterrupt(); 
      applyOutputs();      // Bắt buộc ESC chạy theo mức PWM mới load
      Serial.println("Config reloaded."); 
      printStatus();       // Gửi status về web để UI đồng bộ lại thanh trượt
    } 
    return; 
  }
  Serial.println("Unknown command. Type help");
}

// ---------------- setup / loop ----------------
static const uint32_t STATUS_PRINT_MS = 250;
uint32_t lastStatusPrintMs = 0;

void setup() {
  #if defined(RTC_CNTL_BROWN_OUT_REG)
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Safe disable Brownout detector
  #endif

  Serial.begin(57600);
  delay(400);

  pinMode(PIN_IGN, OUTPUT); pinMode(PIN_VALVE_1, OUTPUT); pinMode(PIN_VALVE_2, OUTPUT);
  pinMode(PIN_RPM, INPUT);
  digitalWrite(PIN_IGN, IGN_ACTIVE_HIGH ? LOW : HIGH);
  digitalWrite(PIN_VALVE_1, VALVE_ACTIVE_HIGH ? LOW : HIGH);
  digitalWrite(PIN_VALVE_2, VALVE_ACTIVE_HIGH ? LOW : HIGH);

  mountSd();
  loadConfigFromSd();
  attachRpmInterrupt();

  escPumpPeriodUs  = escAttach(PIN_ESC_PUMP, LEDC_CH_PUMP);
  escStartPeriodUs = escAttach(PIN_ESC_START, LEDC_CH_START);

  // Arm ESCs
  escWriteUs(PIN_ESC_PUMP, LEDC_CH_PUMP, ESC_ARM_US, escPumpPeriodUs);
  escWriteUs(PIN_ESC_START, LEDC_CH_START, ESC_ARM_US, escStartPeriodUs);
  delay(4000);
  allOff();

  bool thermoOk = thermo.begin();
  thermo.setFaultChecks(MAX31855_FAULT_ALL);

  Serial.println("ECU Manual V1 booted (Serial-only, fully manual).");
  Serial.print("MAX31855 begin() = "); Serial.println(thermoOk ? "OK" : "CHECK_WIRING");
}

void loop() {
  processSerialRx();
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
    if (rpmDetailMode) printRpmDetail();
  }
  printstatus();
}