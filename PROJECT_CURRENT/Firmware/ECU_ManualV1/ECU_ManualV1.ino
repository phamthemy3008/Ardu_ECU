#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Preferences.h>
#include <math.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <ArduinoOTA.h>
#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif
#include <Adafruit_MAX31855.h>
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"
#include "driver/gpio.h"

// Forward declarations & Global variables needed across functions
extern bool sdMounted;
extern void allOff();
extern void sendWebStatus();
extern void sdLogEvent(const char* evt);
extern void sdLogEvent(const String& msg);
extern int pumpUs;
extern int startUs;
extern const char* rpmEdgeName();
extern char otaUrl[128];
extern SemaphoreHandle_t sdMutex;

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
#define PIN_LED         2  // Status LED (Onboard NodeMCU-32S)
#define PIN_BUTTON     22  // User Button
#define PIN_RC_1       34  // RC Input 1
#define PIN_RC_2       35  // RC Input 2

enum RpmNoiseLevel : uint8_t { RPM_CLEAN, RPM_WARN, RPM_NOISY, RPM_REST_NOISE, RPM_NO_SIGNAL };
static const bool IGN_ACTIVE_HIGH   = true;
static const bool VALVE_ACTIVE_HIGH = true;

// ---------------- Safety Limits Config ----------------
float pegtLimit = 740.0f;       // PEGT 3s forecast limit (°C)
float egtMaxLimit = 800.0f;     // Direct EGT max limit (°C)
float maxRpmLimit = 160000.0f;  // Maximum RPM safety threshold

// ---------------- ESC PWM (raw LEDC) ----------------
static const int ESC_SAFE_US = 1000;   
static const int ESC_MIN_US  = 1000;   
static const int ESC_MAX_US  = 2000;   
static const int ESC_ARM_US  = 1000;    
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

volatile uint32_t rpmMinPulseUs = 450; // Chặn xung rác > 133,000 RPM giả (khuyến nghị chẩn đoán AI)
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
volatile uint32_t isrHist[7] = {0, 0, 0, 0, 0, 0, 0};
volatile uint8_t  isrHistIdx = 0;

struct RpmState {
  float rpm = 0.0f, rpmStage1 = 0.0f, rpmWindow = 0.0f, rpmPeriod = 0.0f, rpmFiltered = 0.0f, rawRawRpm = 0.0f;
  float prevRawRpmCandidate = 0.0f;  // Rate Limiter: lưu RPM thô chu kỳ trước
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

// ---------------- Adaptive PWM→RPM Learning ----------------
// Tự học mối quan hệ giữa xung PWM gửi cho Starter và RPM thực tế.
// Khi tín hiệu sạch (CLEAN/WARN): ghi nhớ cặp (startUs, RPM).
// Khi nhiễu cao: dùng dữ liệu đã học để reject RPM bất hợp lý.
static const int PWM_BIN_WIDTH = 50;       // Mỗi bin rộng 50µs
static const int PWM_BIN_COUNT = 20;       // 20 bins: 1000-1050, 1050-1100, ..., 1950-2000
static const uint16_t PWM_BIN_MIN_TRUST = 10;  // Cần tối thiểu 10 sample mới tin

struct PwmRpmBin {
  float rpmEstimate = 0.0f;   // RPM trung bình đã học
  uint16_t sampleCount = 0;   // Số lần sample (tối đa 10000)
};

PwmRpmBin starterRpmMap[PWM_BIN_COUNT];
uint8_t   starterLearnedBins = 0;  // Đếm số bin đã học đủ (hiển thị trên Web)

int pwmToBinIndex(int us) {
  int idx = (us - ESC_MIN_US) / PWM_BIN_WIDTH;
  if (idx < 0) return 0;
  if (idx >= PWM_BIN_COUNT) return PWM_BIN_COUNT - 1;
  return idx;
}

void learnStarterRpm(int starterPwmUs, float measuredRpm) {
  int idx = pwmToBinIndex(starterPwmUs);

  // === RÀNG BUỘC ĐƠN ĐIỆU (Monotonicity): Không học nếu vi phạm vật lý ===
  // PWM cao hơn PHẢI cho RPM cao hơn. Nếu RPM đo được thấp hơn 80% của bin PWM thấp hơn → nhiễu.
  float rpmFloor = getMonotonicFloor(idx);
  if (rpmFloor > 0.0f && measuredRpm < rpmFloor * 0.80f) {
    return; // Từ chối học giá trị vi phạm đơn điệu
  }

  PwmRpmBin &bin = starterRpmMap[idx];
  if (bin.sampleCount == 0) {
    bin.rpmEstimate = measuredRpm;
  } else {
    // EMA chậm (alpha=0.03): học ổn định, không bị nhiễu nhẹ kéo lệch
    bin.rpmEstimate = 0.03f * measuredRpm + 0.97f * bin.rpmEstimate;
  }
  bool wasTrusted = (bin.sampleCount >= PWM_BIN_MIN_TRUST);
  if (bin.sampleCount < 10000) bin.sampleCount++;
  // Chỉ cập nhật counter khi bin vừa vượt ngưỡng tin cậy (tránh loop 20 bin mỗi lần)
  if (!wasTrusted && bin.sampleCount >= PWM_BIN_MIN_TRUST) starterLearnedBins++;
}

// Lấy RPM sàn từ định luật đơn điệu: RPM tối thiểu ở bin hiện tại = max RPM của tất cả bin thấp hơn
float getMonotonicFloor(int binIdx) {
  float floor = 0.0f;
  for (int i = 0; i < binIdx; i++) {
    if (starterRpmMap[i].sampleCount >= PWM_BIN_MIN_TRUST) {
      if (starterRpmMap[i].rpmEstimate > floor) floor = starterRpmMap[i].rpmEstimate;
    }
  }
  return floor;
}

// Lấy RPM kỳ vọng cho một mức PWM, có nội suy từ bin lân cận
float getExpectedStarterRpm(int starterPwmUs) {
  int idx = pwmToBinIndex(starterPwmUs);
  PwmRpmBin &bin = starterRpmMap[idx];
  if (bin.sampleCount >= PWM_BIN_MIN_TRUST) return bin.rpmEstimate;

  // Nội suy từ bin lân cận gần nhất
  // Tìm bin bên trái có dữ liệu
  int leftIdx = -1;
  for (int i = idx - 1; i >= 0; i--) {
    if (starterRpmMap[i].sampleCount >= PWM_BIN_MIN_TRUST) { leftIdx = i; break; }
  }
  // Tìm bin bên phải có dữ liệu
  int rightIdx = -1;
  for (int i = idx + 1; i < PWM_BIN_COUNT; i++) {
    if (starterRpmMap[i].sampleCount >= PWM_BIN_MIN_TRUST) { rightIdx = i; break; }
  }

  if (leftIdx >= 0 && rightIdx >= 0) {
    // Nội suy tuyến tính giữa 2 bin
    float t = (float)(idx - leftIdx) / (float)(rightIdx - leftIdx);
    return starterRpmMap[leftIdx].rpmEstimate + t * (starterRpmMap[rightIdx].rpmEstimate - starterRpmMap[leftIdx].rpmEstimate);
  } else if (leftIdx >= 0) {
    // KHÔNG extrapolate ngang (sẽ ghim RPM thấp khi PWM tăng). Trả về -1 để bỏ qua Phase 3.
    return -1.0f;
  } else if (rightIdx >= 0) {
    return -1.0f;
  }
  return -1.0f; // Chưa có dữ liệu nào
}

void resetLearnedRpmMap() {
  for (int i = 0; i < PWM_BIN_COUNT; i++) {
    starterRpmMap[i].rpmEstimate = 0.0f;
    starterRpmMap[i].sampleCount = 0;
  }
  starterLearnedBins = 0;
}

void printLearnedRpmMap() {
  Serial.println("===== ADAPTIVE PWM->RPM MAP =====");
  Serial.print("Bins learned: "); Serial.print(starterLearnedBins); Serial.print("/"); Serial.println(PWM_BIN_COUNT);
  for (int i = 0; i < PWM_BIN_COUNT; i++) {
    int lo = ESC_MIN_US + i * PWM_BIN_WIDTH;
    int hi = lo + PWM_BIN_WIDTH;
    Serial.print("  ["); Serial.print(lo); Serial.print("-"); Serial.print(hi); Serial.print("us] ");
    if (starterRpmMap[i].sampleCount >= PWM_BIN_MIN_TRUST) {
      Serial.print("RPM="); Serial.print(starterRpmMap[i].rpmEstimate, 0);
      Serial.print(" (n="); Serial.print(starterRpmMap[i].sampleCount); Serial.print(")");
    } else if (starterRpmMap[i].sampleCount > 0) {
      Serial.print("learning... ("); Serial.print(starterRpmMap[i].sampleCount); Serial.print("/"); Serial.print(PWM_BIN_MIN_TRUST); Serial.print(")");
    } else {
      Serial.print("--");
    }
    Serial.println();
  }
  Serial.println("=================================");
}

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

// ---------------- Output state & Ramp Control ----------------
int pumpUs = ESC_SAFE_US, startUs = ESC_SAFE_US;
int startSetUs = 1200, pumpSetUs = 1200;
int startStepUs = 10, pumpStepUs = 10;
bool ignCmd = false, valve1Cmd = false, valve2Cmd = false;

// Ramp Settings
bool pumpRampEnabled = true;
bool starterRampEnabled = true;
float pumpRampDuration = 1.5f;     // 1.5 giây
float starterRampDuration = 1.0f;  // 1.0 giây
float starterRampK = 1.2f;

int pumpTargetUs = ESC_SAFE_US, pumpStartUs = ESC_SAFE_US;
uint32_t pumpRampStartMs = 0;

int startTargetUs = ESC_SAFE_US, startStartUs = ESC_SAFE_US;
uint32_t startRampStartMs = 0;

// 1. HÀM BƠM NHIÊN LIỆU: S-Curve (Smoothstep) 3x^2 - 2x^3
float CalculatePumpSymmetricRamp(float current_time, float ramp_duration, float p_start, float p_target) {
    if (current_time <= 0.0f) return p_start;
    if (current_time >= ramp_duration) return p_target;
    float x = current_time / ramp_duration;
    float s_curve = (3.0f * x * x) - (2.0f * x * x * x);
    return p_start + (p_target - p_start) * s_curve;
}

// 2. HÀM MÔ-TƠ ĐỀ: Gia tốc Mũ (Exponential Ramp Up)
float CalculateStarterRampUp(float current_time, float ramp_duration, float s_min, float s_max, float k) {
    if (current_time <= 0.0f) return s_min;
    if (current_time >= ramp_duration) return s_max;
    float exp_factor = 1.0f - expf(-k * current_time);
    float max_factor = 1.0f - expf(-k * ramp_duration);
    if (max_factor < 1e-6f) return s_max;
    float normalized_factor = exp_factor / max_factor;
    return s_min + (s_max - s_min) * normalized_factor;
}

void setPumpTargetUs(int targetUs) {
  targetUs = constrain(targetUs, ESC_MIN_US, ESC_MAX_US);
  if (targetUs != pumpTargetUs) {
    pumpStartUs = pumpUs;
    pumpTargetUs = targetUs;
    pumpRampStartMs = millis();
  }
}

void setStartTargetUs(int targetUs) {
  targetUs = constrain(targetUs, ESC_MIN_US, ESC_MAX_US);
  if (targetUs != startTargetUs) {
    startStartUs = startUs;
    startTargetUs = targetUs;
    startRampStartMs = millis();
  }
}

void updateRamps() {
  uint32_t now = millis();

  // BƠM RAMP (S-Curve)
  if (pumpRampEnabled && pumpUs != pumpTargetUs) {
    float elapsedS = (float)(now - pumpRampStartMs) / 1000.0f;
    if (elapsedS >= pumpRampDuration) {
      pumpUs = pumpTargetUs;
    } else {
      pumpUs = (int)roundf(CalculatePumpSymmetricRamp(elapsedS, pumpRampDuration, (float)pumpStartUs, (float)pumpTargetUs));
    }
  } else if (!pumpRampEnabled) {
    pumpUs = pumpTargetUs;
  }

  // ĐỀ RAMP (Exponential)
  if (starterRampEnabled && startUs != startTargetUs) {
    float elapsedS = (float)(now - startRampStartMs) / 1000.0f;
    if (elapsedS >= starterRampDuration) {
      startUs = startTargetUs;
    } else {
      startUs = (int)roundf(CalculateStarterRampUp(elapsedS, starterRampDuration, (float)startStartUs, (float)startTargetUs, starterRampK));
    }
  } else if (!starterRampEnabled) {
    startUs = startTargetUs;
  }
}

bool allOutputsOff() {
  return startUs <= ESC_SAFE_US && pumpUs <= ESC_SAFE_US && !ignCmd && !valve1Cmd && !valve2Cmd;
}

bool rpmAtRestGuardCondition() {
  if (rpmCalMode) return false;
  if (!allOutputsOff()) return false;
  // Chỉ kích hoạt Rest Guard khi tất cả output OFF VÀ động cơ đã dừng hẳn (< 500 RPM)
  // Giúp động cơ tự quay/coasting không bị ngắt RPM về 0 đột ngột khi vừa tắt Đề
  if (rpmData.rpmFiltered > 500.0f || rpmData.rpm > 500.0f) return false;
  return true;
}

RpmNoiseLevel classifyRpmNoise(bool recent, uint32_t raw, uint32_t accepted, uint32_t rejected,
                               float rejectPct, float jitterPct, float rpmDiffPct, uint32_t intervals) {
  if (!recent && raw == 0) return RPM_NO_SIGNAL;
  if (raw > 1 && accepted == 0) return RPM_NOISY;
  
  // Nếu tín hiệu lọt qua mask RẤT ỔN ĐỊNH (jitter thấp), chứng tỏ Mask đã lọc thành công nhiễu đồng bộ (vd: từ chổi than).
  // Ta không nên phạt quá nặng (NOISY) chỉ vì rejectPct cao.
  bool maskSuccessful = (intervals >= 3 && jitterPct < 15.0f && rpmDiffPct < 15.0f);

  if (!maskSuccessful && (rejected >= 3 || rejectPct > 20.0f || (intervals >= 5 && jitterPct > 30.0f) || rpmDiffPct > 30.0f)) {
    return RPM_NOISY;
  }
  if (rejected > 0 || rejectPct > 5.0f || (intervals >= 3 && jitterPct > 15.0f) || rpmDiffPct > 15.0f) {
    return RPM_WARN;
  }
  return RPM_CLEAN;
}

String serialCmdBuf = "";
unsigned long lastSerialRxTime = 0;
bool webChecksumMode = false; // Locks on after first valid checksum from Web

// CRC-8 (polynomial 0x07) - industry standard error detection
uint8_t crc8(const char* data, unsigned int len) {
  uint8_t crc = 0;
  for (unsigned int i = 0; i < len; i++) {
    crc ^= (uint8_t)data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x07;
      else crc = crc << 1;
    }
  }
  return crc;
}

void processSerialRx() {
  if (serialCmdBuf.length() > 0 && millis() - lastSerialRxTime > 50) {
    serialCmdBuf = "";
  }

  while (Serial.available()) {
    lastSerialRxTime = millis();
    char c = (char)Serial.read();

    if ((c < 32 || c > 126) && c != '\n' && c != '\r') {
      continue; 
    }

    if (c == '\n') {
      if (serialCmdBuf.length() > 0) {
        // Check for checksum: command#XX
        int hashIdx = serialCmdBuf.lastIndexOf('#');
        if (hashIdx > 0 && (serialCmdBuf.length() - hashIdx) == 3) {
          // Has checksum format - validate CRC-8
          String cmdPart = serialCmdBuf.substring(0, hashIdx);
          String chkPart = serialCmdBuf.substring(hashIdx + 1);
          uint8_t expected = (uint8_t)strtol(chkPart.c_str(), NULL, 16);
          uint8_t actual = crc8(cmdPart.c_str(), cmdPart.length());
          if (actual == expected) {
            webChecksumMode = true;
            Serial.print("-> [RX OK]: ");
            Serial.println(cmdPart);
            handleCommand(cmdPart);
          } else {
            Serial.print("-> [RX NOISE DROPPED]: ");
            Serial.println(serialCmdBuf);
          }
        } else if (webChecksumMode) {
          // In checksum mode but # was corrupted or missing - reject
          Serial.print("-> [RX NOISE DROPPED]: ");
          Serial.println(serialCmdBuf);
        } else {
          // No checksum mode yet (manual Serial Monitor) - accept as-is
          Serial.print("-> [RX Received]: ");
          Serial.println(serialCmdBuf);
          handleCommand(serialCmdBuf);
        }
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
    if (isrLastPeriodUs > 0 && isrLastPeriodUs <= 100000UL) {
      uint32_t dynMaskUs = (isrLastPeriodUs * 65UL) / 100UL; // Siết chặt mặt nạ động 65% để lọc bỏ đột biến > 65% Delta
      if (dynMaskUs > 50000UL) dynMaskUs = 50000UL;
      if (dynMaskUs > maskUs) maskUs = dynMaskUs;
    }
    if (dtAcceptedUs < maskUs) { isrRejectedEdges++; return; }
    
    // Median Filter 7 phần tử — loại bỏ các chùm gai nhiễu liên tiếp (burst noise)
    if (isrHist[0] == 0) {
      for (uint8_t i = 0; i < 7; i++) isrHist[i] = dtAcceptedUs;
    } else {
      isrHist[isrHistIdx] = dtAcceptedUs;
      isrHistIdx = (isrHistIdx + 1) % 7;
    }
    // Sắp xếp tìm trung vị 7 phần tử tối ưu trong IRAM
    uint32_t s[7];
    for (uint8_t i = 0; i < 7; i++) s[i] = isrHist[i];
    for (uint8_t i = 1; i < 7; i++) {
      uint32_t key = s[i];
      int8_t j = i - 1;
      while (j >= 0 && s[j] > key) {
        s[j + 1] = s[j];
        j--;
      }
      s[j + 1] = key;
    }
    uint32_t medianUs = s[3]; // Phần tử trung vị chính xác (median)

    isrLastPeriodUs = medianUs; // Dùng median thay vì dtAcceptedUs để không bị lock-out mask khi rớt 1 xung
    isrAcceptedIntervals++;
    isrSumDtUs += medianUs; // Cộng median thay vì raw để tính trung bình tuyệt đối tĩnh
    isrSumDtSqUs += (uint64_t)medianUs * (uint64_t)medianUs;
    if (medianUs < isrMinDtUs) isrMinDtUs = medianUs;
    if (medianUs > isrMaxDtUs) isrMaxDtUs = medianUs;
  }
  isrLastAcceptedPulseUs = nowUs;
  isrAcceptedPulses++;
}

void resetRpmStats() {
  noInterrupts();
  isrLastRawEdgeUs = 0; isrLastAcceptedPulseUs = 0; isrLastPeriodUs = 0;
  isrRawEdges = 0; isrAcceptedPulses = 0; isrRejectedEdges = 0; isrAcceptedIntervals = 0;
  isrSumDtUs = 0; isrSumDtSqUs = 0; isrMinDtUs = 0xFFFFFFFFUL; isrMaxDtUs = 0;
  for (uint8_t i = 0; i < 7; i++) isrHist[i] = 0; isrHistIdx = 0;
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
  rpmData.rawRawRpm = (raw > 0 && windowUs > 0) ?
                      ((float)raw * 60000000.0f / ((float)windowUs * (float)pulsesPerRev)) : 0.0f;
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
    rpmData.rpm = 0.0f; rpmData.rpmFiltered = 0.0f; rpmData.signalRecent = false; rpmData.noise = RPM_REST_NOISE;
  } else {
    rpmData.noise = classifyRpmNoise(rpmData.signalRecent, raw, accepted, rejected,
                                     rpmData.rejectPct, rpmData.jitterPct, rpmData.rpmDiffPct, validN);
    
    // Theo dõi giai đoạn chuyển tiếp khi vừa ngắt Mô-tơ Đề (2 giây đầu sau khi tắt Đề)
    static uint32_t lastStarterActiveMs = 0;
    if (startUs > ESC_SAFE_US) {
      lastStarterActiveMs = nowMs;
    }
    bool inStarterTransition = (nowMs - lastStarterActiveMs < 2000);

    // Dynamic Outlier Guard: Ngưỡng trần linh hoạt theo RPM hiện tại thay vì nhảy vọt 160k
    float maxAllowedRpm;
    if (rpmData.rpmFiltered < 5000.0f) {
      maxAllowedRpm = (startUs > ESC_SAFE_US || inStarterTransition) ? 15000.0f : 20000.0f;
    } else if (rpmData.rpmFiltered < 25000.0f) {
      maxAllowedRpm = max(30000.0f, rpmData.rpmFiltered * 1.8f);
    } else {
      maxAllowedRpm = max(50000.0f, min(160000.0f, rpmData.rpmFiltered * 1.6f));
    }

    // Tính RPM trung bình vi-mô từ tổng thời gian các xung hợp lệ trong 100ms (Microsecond Average RPM)
    float rawRpmCandidate = 0.0f;
    if (validN > 0 && rpmData.avgIntervalUs > 0.0f) {
      rawRpmCandidate = 60000000.0f / (rpmData.avgIntervalUs * (float)pulsesPerRev);
    } else if (rpmData.signalRecent && rpmData.rpmPeriod > 0.0f) {
      rawRpmCandidate = rpmData.rpmPeriod;
    } else if (accepted > 0) {
      rawRpmCandidate = rpmData.rpmWindow;
    }

    if (rawRpmCandidate > maxAllowedRpm) {
      rawRpmCandidate = (rpmData.rpmWindow > 0.0f && rpmData.rpmWindow <= maxAllowedRpm) ? rpmData.rpmWindow : 0.0f;
    }

    // --- RATE LIMITER: Giới hạn tốc độ thay đổi RPM (slew rate) ---
    // Turbine jet không thể tăng/giảm quá nhanh khi quay không tải.
    // Khi đang đốt dầu (pumpUs > ESC_SAFE_US || egt.c > 80°C), gia tốc spool-up thực tế của turbine có thể lên tới 35,000 RPM/s (3500 RPM/100ms)
    bool activeCombustion = (pumpUs > ESC_SAFE_US || (egt.ok && egt.c > 80.0f));
    if (rpmData.prevRawRpmCandidate > 0.0f && rawRpmCandidate > 0.0f) {
      float maxSlew = (inStarterTransition || (startUs > ESC_SAFE_US && !activeCombustion)) ? 1200.0f : ((rpmData.prevRawRpmCandidate < 30000.0f) ? 3500.0f : 12000.0f);
      float delta = rawRpmCandidate - rpmData.prevRawRpmCandidate;
      if (delta > maxSlew) rawRpmCandidate = rpmData.prevRawRpmCandidate + maxSlew;
      else if (delta < -maxSlew) rawRpmCandidate = rpmData.prevRawRpmCandidate - maxSlew;
    }
    rpmData.prevRawRpmCandidate = rawRpmCandidate;

    // --- ADAPTIVE PWM-AWARE FILTER: Lọc dựa trên mô hình vật lý ---
    // Nếu starter đang chạy hoặc mới ngắt Đề: dùng dữ liệu đã học để loại bỏ RPM bất hợp lý
    // CHÚ Ý: Khi động cơ đang đốt dầu (activeCombustion), tuabin sinh công kéo tua tăng nhanh hơn motor đề,
    // KHÔNG được kẹp trần hi theo motor đề để tuabin tự do spool-up lên Idle!
    if (startUs > ESC_SAFE_US || inStarterTransition) {
      int currentBinIdx = (startUs > ESC_SAFE_US) ? pwmToBinIndex(startUs) : pwmToBinIndex(1100);

      // PHASE 1 - HỌC: Chỉ học khi chạy Đề thuần túy (CHƯA bơm nhiên liệu và CHƯA nóng buồng đốt)
      bool stableForLearning = (rpmData.noise <= RPM_WARN) || (rpmData.noise == RPM_NOISY && rpmData.jitterPct < 20.0f && rpmData.validIntervals >= 2);
      
      if (startUs > ESC_SAFE_US && !activeCombustion && stableForLearning && rawRpmCandidate > 100.0f) {
        learnStarterRpm(startUs, rawRpmCandidate);
      }

      // PHASE 2 - LỌC: Ràng buộc đơn điệu (Monotonicity Floor)
      float rpmFloor = getMonotonicFloor(currentBinIdx);
      if (rpmFloor > 0.0f && rawRpmCandidate > 0.0f && rawRpmCandidate < rpmFloor * 0.80f) {
        float snapTarget = (getExpectedStarterRpm(startUs) > 0.0f) ? getExpectedStarterRpm(startUs) : rpmFloor;
        rawRpmCandidate = snapTarget;
        rpmData.prevRawRpmCandidate = snapTarget;
      }

      // PHASE 3 - LỌC: Tolerance window từ dữ liệu đã học
      float expectedRpm = getExpectedStarterRpm(startUs);
      if (expectedRpm > 0.0f && rawRpmCandidate > 0.0f) {
        uint16_t n = starterRpmMap[currentBinIdx].sampleCount;
        float tolerance = (n < PWM_BIN_MIN_TRUST) ? 0.50f : max(0.25f, 0.50f - (float)(n - PWM_BIN_MIN_TRUST) * 0.001f);
        float lo = expectedRpm * (1.0f - tolerance);
        float hi = expectedRpm * (1.0f + tolerance);
        if (rpmFloor > 0.0f && lo < rpmFloor * 0.80f) lo = rpmFloor * 0.80f;
        
        if (!activeCombustion) {
          // Khi chỉ quay bằng motor đề: Kẹp cả trần trên và sàn dưới
          if (rawRpmCandidate < lo || rawRpmCandidate > hi) {
            rawRpmCandidate = expectedRpm;
            rpmData.prevRawRpmCandidate = expectedRpm;
          }
        } else {
          // Khi đang đốt dầu sinh công: Chỉ kẹp sàn dưới chống rớt ảo, CHO PHÉP vượt trần hi để spool-up
          if (rawRpmCandidate < lo * 0.75f) {
            rawRpmCandidate = expectedRpm;
            rpmData.prevRawRpmCandidate = expectedRpm;
          }
        }
      }
    }

    // --- CASCADED DUAL-EMA bậc 2: Lọc 2 tầng nối tiếp ---
    // Chọn hệ số lọc mềm mại dựa trên trạng thái Đề, Chuyển tiếp & Nhiễu (khuyến nghị chẩn đoán AI: alpha 0.18 giúp FRPM mịn tuyệt đối)
    float alpha1, alpha2, alphaTrigger;
    if (startUs > ESC_SAFE_US || inStarterTransition) {
      alpha1 = 0.18f; alpha2 = 0.35f; alphaTrigger = 0.12f; // Đề / chuyển tiếp: Lọc cực mịn, triệt tiêu gai nhiễu EMI
    } else if (rpmData.noise >= RPM_WARN || rpmData.rpmFiltered < 25000.0f) {
      alpha1 = 0.28f; alpha2 = 0.48f; alphaTrigger = 0.22f; // Tự quay dải thấp / nhiễu: Lọc vừa
    } else {
      alpha1 = 0.45f; alpha2 = 0.65f; alphaTrigger = 0.40f; // Tự quay dải cao & sạch: Lọc nhanh
    }

    if (rpmData.rpmStage1 == 0.0f && rawRpmCandidate > 0.0f) {
      rpmData.rpmStage1 = rawRpmCandidate;
      rpmData.rpm = rawRpmCandidate;
    } else {
      rpmData.rpmStage1 = (alpha1 * rawRpmCandidate) + ((1.0f - alpha1) * rpmData.rpmStage1);
      rpmData.rpm = (alpha2 * rpmData.rpmStage1) + ((1.0f - alpha2) * rpmData.rpm);
    }
    if (rawRpmCandidate == 0.0f && rpmData.rpm < 50.0f) {
      rpmData.rpmStage1 = 0.0f;
      rpmData.rpm = 0.0f;
    }

    // --- BIẾN RPM LỌC NHIỄU AN TOÀN CHO TRIGGER (rpmFiltered) ---
    float rawTriggerTarget = (rpmData.rpm <= maxAllowedRpm) ? rpmData.rpm : 0.0f;

    if (rpmData.rpmFiltered == 0.0f && rawTriggerTarget > 0.0f) {
      rpmData.rpmFiltered = rawTriggerTarget;
    } else {
      rpmData.rpmFiltered = (alphaTrigger * rawTriggerTarget) + ((1.0f - alphaTrigger) * rpmData.rpmFiltered);
    }
    if (rawTriggerTarget == 0.0f || rpmData.rpm == 0.0f) {
      rpmData.rpmFiltered = 0.0f;
    }

    // --- BẢO VỆ OVERREV (RPM > maxRpmLimit) ---
    if (pumpUs > ESC_SAFE_US && rpmData.rpmFiltered > maxRpmLimit) {
      emergencyStop();
      Serial.print("ERR:E06|RPM="); Serial.println(rpmData.rpmFiltered, 0);
      sendWebStatus();
    }
  }
}

// ---------------- EGT (MAX31855) ----------------
static const uint32_t EGT_READ_PERIOD_MS = 120;
Adafruit_MAX31855 thermo(PIN_EGT_CLK, PIN_EGT_CS, PIN_EGT_DO);

struct EgtState {
  bool ok = false;
  float c = NAN, prevC = NAN, gradientCps = 0.0f, cProjected3s = NAN;
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
    if (dtS > 0.05f) {
      float rawGrad = ((float)tc - egt.c) / dtS;
      // Khi ngắt Đề hoặc động cơ đang dừng không bơm nhiên liệu (pumpUs <= ESC_SAFE_US),
      // sụt áp có thể gây xung dEGT ảo (+15°C/s). Khống chế dEGT ảo khi chưa bơm nhiên liệu
      if (pumpUs <= ESC_SAFE_US && fabsf(rawGrad) > 25.0f) {
        rawGrad = (rawGrad > 0) ? 8.0f : -8.0f;
      }
      egt.gradientCps = rawGrad;
      egt.cProjected3s = (float)tc + (egt.gradientCps * 3.0f);

      // --- BẢO VỆ CHỐNG BÙNG NHIỆT (PEGT 3s > pegtLimit hoặc EGT > egtMaxLimit) ---
      // Chỉ ngắt khẩn cấp do PEGT khi bơm xăng đang chạy VÀ nhiệt độ EGT thực tế đã bắt đầu nóng (> 100°C)
      if (pumpUs > ESC_SAFE_US && ((egt.c > 100.0f && egt.cProjected3s > pegtLimit) || egt.c > egtMaxLimit)) {
        emergencyStop();
        Serial.print("ERR:E02|PEGT="); Serial.print(egt.cProjected3s, 1);
        Serial.print("|EGT="); Serial.println(egt.c, 1);
        sendWebStatus();
      }
    }
  } else {
    egt.gradientCps = 0;
    egt.cProjected3s = NAN;
  }
  egt.prevC = egt.c; egt.c = (float)tc; egt.ok = true; egt.fault = 0; egt.lastGoodMs = nowMs;
}

// ---------------- SD config (Chuyển sang HSPI để không đụng VSPI của MAX31855) ----------------
SPIClass sdSPI(HSPI);
bool sdOk = false, sdMounted = false;
static const char* CONFIG_PATH = "/ECUCFG.TXT";

bool mountSd(bool force = false) {
  if (sdMounted && sdOk && !force) return true;

  sdOk = false;
  sdMounted = false;

  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);

  sdSPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  delay(20);

  // Thử 4MHz trước, nếu không được thử 1MHz
  if (!SD.begin(PIN_SD_CS, sdSPI, 4000000)) {
    if (!SD.begin(PIN_SD_CS, sdSPI, 1000000)) {
      Serial.println("SD: begin() FAIL.");
      return false;
    }
  }

  if (SD.cardType() == CARD_NONE) {
    Serial.println("SD: no card inserted.");
    return false;
  }

  sdOk = true;
  sdMounted = true;
  Serial.println("SD: Mounted OK.");
  return true;
}

static int clampInt(long v, long lo, long hi) { return (int)(v < lo ? lo : (v > hi ? hi : v)); }

// ---------------- ESP32 Preferences / NVS Config Persistence ----------------
Preferences preferences;

char wifiSsid[32] = "";
char wifiPass[64] = "";
bool wifiConnecting = false;
bool wifiScanRequested = false;
unsigned long wifiScanStartMs = 0;

void connectWifi() {
  if (strlen(wifiSsid) == 0) {
    Serial.println("WIFI: Cannot connect - SSID empty.");
    return;
  }
  Serial.printf("WIFI: Connecting to '%s'...\n", wifiSsid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid, wifiPass);
  wifiConnecting = true;
}

void saveConfigToNvs() {
  preferences.begin("ecu_cfg", false);
  preferences.putFloat("pegtLimit", pegtLimit);
  preferences.putFloat("egtMaxLimit", egtMaxLimit);
  preferences.putFloat("maxRpmLimit", maxRpmLimit);
  preferences.putUChar("ppr", pulsesPerRev);
  preferences.putUInt("rpmfilter", rpmMinPulseUs);
  preferences.putUChar("rpmedge", (uint8_t)rpmEdgeMode);
  preferences.putUShort("startset", startSetUs);
  preferences.putUShort("pumpset", pumpSetUs);
  preferences.putUChar("startstep", startStepUs);
  preferences.putUChar("pumpstep", pumpStepUs);
  preferences.putString("wifissid", wifiSsid);
  preferences.putString("wifipass", wifiPass);
  preferences.putString("otaurl", otaUrl);
  preferences.end();
  Serial.println("NVS: Saved safety & setup config to ESP32 Flash NVS.");
}

bool loadConfigFromNvs() {
  preferences.begin("ecu_cfg", true);
  if (!preferences.isKey("pegtLimit")) {
    preferences.end();
    Serial.println("NVS: No saved config found in flash NVS.");
    return false;
  }
  pegtLimit     = preferences.getFloat("pegtLimit", 740.0f);
  egtMaxLimit   = preferences.getFloat("egtMaxLimit", 800.0f);
  maxRpmLimit   = preferences.getFloat("maxRpmLimit", 160000.0f);
  pulsesPerRev  = preferences.getUChar("ppr", 1);
  rpmMinPulseUs = preferences.getUInt("rpmfilter", 450);
  rpmEdgeMode   = preferences.getUChar("rpmedge", RISING);
  startSetUs    = preferences.getUShort("startset", 1300);
  pumpSetUs     = preferences.getUShort("pumpset", 1200);
  startStepUs   = preferences.getUChar("startstep", 10);
  pumpStepUs    = preferences.getUChar("pumpstep", 10);
  String wSsid  = preferences.getString("wifissid", "");
  String wPass  = preferences.getString("wifipass", "");
  String oUrl   = preferences.getString("otaurl", "http://domain-cua-ban.com/firmware.bin");
  wSsid.toCharArray(wifiSsid, sizeof(wifiSsid));
  wPass.toCharArray(wifiPass, sizeof(wifiPass));
  oUrl.toCharArray(otaUrl, sizeof(otaUrl));
  preferences.end();
  Serial.println("NVS: Config loaded from ESP32 Flash NVS.");
  return true;
}

bool saveConfigToSd() {
  saveConfigToNvs(); // Synchronize NVS flash whenever SD config is saved
  if (sdMutex && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    Serial.println("SAVECFG: SD bus busy.");
    return false;
  }
  
  bool success = false;
  if (mountSd()) {
    if (SD.exists(CONFIG_PATH)) {
      SD.remove(CONFIG_PATH);
    }
    
    File f = SD.open(CONFIG_PATH, FILE_WRITE);
    if (f) {
      f.println("# ECU Manual V1 sensor setup - auto-loaded on boot");
      f.print("ppr=");        f.println((int)pulsesPerRev);
      f.print("rpmfilter="); f.println((uint32_t)rpmMinPulseUs);
      f.print("rpmedge=");   f.println(rpmEdgeName());
      f.print("startset=");  f.println(startSetUs);
      f.print("pumpset=");   f.println(pumpSetUs);
      f.print("startstep="); f.println(startStepUs);
      f.print("pumpstep=");  f.println(pumpStepUs);
      f.print("pump=");      f.println(pumpUs);
      f.print("start=");     f.println(startUs);
      f.print("pegtmax=");   f.println(pegtLimit, 1);
      f.print("egtmax=");    f.println(egtMaxLimit, 1);
      f.print("maxrpm=");    f.println(maxRpmLimit, 0);
      f.print("wifissid=");  f.println(wifiSsid);
      f.print("wifipass=");  f.println(wifiPass);
      f.print("otaurl=");    f.println(otaUrl);
      f.flush();
      f.close();
      Serial.println("SAVECFG: OK -> /ECUCFG.TXT");
      success = true;
    } else {
      Serial.println("SAVECFG: open FILE_WRITE FAIL.");
    }
  } else {
    Serial.println("SAVECFG: saved to NVS, but no SD card.");
    success = true;
  }

  if (sdMutex) xSemaphoreGive(sdMutex);
  return success;
}

bool loadConfigFromSd() {
  if (sdMutex && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    Serial.println("LOADCFG: SD bus busy.");
    return false;
  }

  bool success = false;
  if (mountSd() && SD.exists(CONFIG_PATH)) {
    File f = SD.open(CONFIG_PATH, FILE_READ);
    if (f) {
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
        else if (key == "startset")  startSetUs = clampInt(n, ESC_MIN_US, ESC_MAX_US);
        else if (key == "pumpset")   pumpSetUs = clampInt(n, ESC_MIN_US, ESC_MAX_US);
        else if (key == "startstep") startStepUs = clampInt(n, 1, 100);
        else if (key == "pumpstep")  pumpStepUs = clampInt(n, 1, 100);
        else if (key == "pump")      pumpUs = clampInt(n, ESC_MIN_US, ESC_MAX_US);
        else if (key == "start")     startUs = clampInt(n, ESC_MIN_US, ESC_MAX_US);
        else if (key == "pegtmax")   pegtLimit = (float)atof(val.c_str());
        else if (key == "egtmax")    egtMaxLimit = (float)atof(val.c_str());
        else if (key == "maxrpm")    maxRpmLimit = (float)atof(val.c_str());
        else if (key == "wifissid")  val.toCharArray(wifiSsid, sizeof(wifiSsid));
        else if (key == "wifipass")  val.toCharArray(wifiPass, sizeof(wifiPass));
        else if (key == "otaurl")    val.toCharArray(otaUrl, sizeof(otaUrl));
        else continue;
        
        applied++;
      }
      f.close();
      Serial.print("LOADCFG: applied "); Serial.print(applied); Serial.println(" keys.");
      success = (applied > 0);
    } else {
      Serial.println("LOADCFG: open FILE_READ FAIL.");
    }
  } else {
    Serial.println("LOADCFG: no SD card or /ECUCFG.TXT, using defaults.");
  }

  if (sdMutex) xSemaphoreGive(sdMutex);
  if (success) sdLogEvent("CONFIG_LOADED");
  return success;
}

// ---------------- SD Telemetry & Event Logging (Async Core 0 Task) ----------------
#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <ArduinoOTA.h>

char otaUrl[192] = "https://raw.githubusercontent.com/phamthemy3008/Adru_ECU/main/PROJECT_CURRENT/Firmware/ECU_ManualV1/firmware.bin";
bool otaEnabled = false;
bool otaInProgress = false;
bool otaBootCheckDone = false;
bool otaUpdatePending = false;
uint32_t otaNoticeStartMs = 0;
static TaskHandle_t otaTaskHandle = NULL;

void checkHttpOtaOnBoot() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("HTTP OTA: WiFi not connected.");
    return;
  }
  if (strlen(otaUrl) == 0) {
    Serial.println("HTTP OTA: Skip check (OTA URL empty).");
    return;
  }

  Serial.print("HTTP OTA: Checking remote server -> ");
  Serial.println(otaUrl);

  HTTPClient http;
  http.begin(otaUrl);
  http.setTimeout(4000); // 4 seconds timeout for check
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK || httpCode == 301 || httpCode == 302) {
    int contentLen = http.getSize();
    http.end();
    Serial.printf("HTTP OTA: New Firmware detected! Size: %d bytes.\n", contentLen);
    Serial.println("HTTP OTA: Prompting user! Waiting 30s for confirmation...");
    
    otaUpdatePending = true;
    otaNoticeStartMs = millis();
    Serial.println("EV:OTA_NEW_VER");
    sendWebStatus();
  } else {
    http.end();
    Serial.printf("HTTP OTA: Server returned code %d (No update or unreachable).\n", httpCode);
    Serial.println("EV:OTA_NO_NEW_VER");
  }
}

void performHttpOtaUpdate() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("HTTP OTA FAIL: WiFi not connected.");
    return;
  }
  if (strlen(otaUrl) == 0) {
    Serial.println("HTTP OTA FAIL: OTA URL empty.");
    return;
  }

  Serial.println("HTTP OTA: Stopping all outputs safely before flash...");
  allOff();
  delay(500);

  Serial.print("HTTP OTA: Downloading & Flashing from "); Serial.println(otaUrl);
  WiFiClient client;
  httpUpdate.rebootOnUpdate(true);

  t_httpUpdate_return ret = httpUpdate.update(client, otaUrl);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP OTA ERROR (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP OTA: No update available on server.");
      break;
    case HTTP_UPDATE_OK:
      Serial.println("HTTP OTA: SUCCESS! Rebooting...");
      break;
  }
}

void updateOtaNotice() {
  if (!otaUpdatePending) return;

  uint32_t elapsed = millis() - otaNoticeStartMs;
  if (elapsed >= 30000) { // 30 seconds confirmation timeout
    otaUpdatePending = false;
    Serial.println("HTTP OTA: 30s Timeout reached! Firmware update canceled.");
    Serial.println("EV:OTA_CANCELED");
    sendWebStatus();
  }
}

void otaTaskWorker(void *pvParameters) {
  for (;;) {
    if (otaEnabled) {
      ArduinoOTA.handle();
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // Polling 50Hz on Core 0
  }
}

void initOtaCore0(const char* hostname = "ECU-JetEngine", const char* password = "") {
  if (otaTaskHandle != NULL) return; // Already initialized

  ArduinoOTA.setHostname(hostname);
  if (password && strlen(password) > 0) {
    ArduinoOTA.setPassword(password);
  }

  ArduinoOTA.onStart([]() {
    otaInProgress = true;
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {
      type = "filesystem";
    }
    Serial.println("OTA: Flash Update Start -> " + type);
    // Safety precaution: when OTA flash begins, cut outputs to safe state
    allOff();
  });

  ArduinoOTA.onEnd([]() {
    otaInProgress = false;
    Serial.println("\nOTA: Flash Update Complete! Rebooting...");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static uint8_t lastPct = 0;
    uint8_t pct = (progress / (total / 100));
    if (pct != lastPct && pct % 10 == 0) {
      lastPct = pct;
      Serial.printf("OTA Progress: %u%%\n", pct);
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    otaInProgress = false;
    Serial.printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  otaEnabled = true;

  xTaskCreatePinnedToCore(
    otaTaskWorker,
    "otaTask",
    4096,
    NULL,
    1, // Low priority on Core 0
    &otaTaskHandle,
    0  // Pin to Core 0 (0% impact on Core 1 engine loop)
  );
  Serial.println("OTA: Initialized & pinned to Async Core 0.");
}

bool sdLoggingEnabled = true;
char sdLogPath[32] = "/ECU000.CSV";
static uint32_t lastSdTelemetryMs = 0;
static const uint32_t SD_LOG_PERIOD_MS = 500; // Ghi Telemetry 2Hz (500ms/mẫu)
static uint32_t sdWriteFailCount = 0;
static uint32_t sdConsecFail = 0;
static const uint32_t SD_MAX_CONSEC_FAIL = 5;

static QueueHandle_t sdQueue = NULL;
SemaphoreHandle_t sdMutex = NULL;
static TaskHandle_t sdTaskHandle = NULL;
static const uint8_t SD_QUEUE_SIZE = 32;

struct SdLogMsg {
  char line[256];
};

void sdTaskWorker(void *pvParameters) {
  SdLogMsg msg;
  for (;;) {
    if (xQueueReceive(sdQueue, &msg, portMAX_DELAY) == pdTRUE) {
      if (!sdOk || !sdLoggingEnabled) continue;

      if (sdMutex && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        File f = SD.open(sdLogPath, FILE_APPEND);
        if (!f) {
          sdWriteFailCount++;
          sdConsecFail++;
          if (sdWriteFailCount == 1 || sdWriteFailCount % 10 == 0) {
            Serial.print("SD LOG WRITE FAIL count="); Serial.println(sdWriteFailCount);
          }
          if (sdConsecFail >= SD_MAX_CONSEC_FAIL) {
            sdOk = false;
            Serial.println("SD LOG: DISABLED (repeated write failures - card removed or corrupt).");
          }
        } else {
          size_t n = f.println(msg.line);
          f.close();
          if (n == 0) {
            sdWriteFailCount++;
            sdConsecFail++;
          } else {
            sdConsecFail = 0;
          }
        }
        xSemaphoreGive(sdMutex);
      }
    }
  }
}

String sdFloat(float v, int decimals) {
  if (isnan(v) || isinf(v)) return "";
  return String(v, decimals);
}

String sdCsvQuote(String s) {
  s.replace("\"", "\"\"");
  return String("\"") + s + "\"";
}

String sdCsvLine(const char* type, const String& eventText) {
  String line;
  line.reserve(256);
  line += String(millis()); line += ",";
  line += type; line += ",";
  line += sdFloat(egt.ok ? egt.c : NAN, 1); line += ",";
  line += sdFloat(egt.ok ? egt.gradientCps : NAN, 1); line += ",";
  line += sdFloat(rpmData.rpm, 0); line += ",";
  line += sdFloat(rpmData.rpmFiltered, 0); line += ",";
  line += sdFloat(rpmData.rawRawRpm, 0); line += ",";
  line += String(pumpUs); line += ",";
  line += String(startUs); line += ",";
  line += (ignCmd ? "1" : "0"); line += ",";
  line += (valve1Cmd ? "1" : "0"); line += ",";
  line += (valve2Cmd ? "1" : "0"); line += ",";
  line += sdCsvQuote(eventText);
  return line;
}

void sdAppendLine(const String& line) {
  if (!sdOk || !sdLoggingEnabled || sdQueue == NULL) return;

  SdLogMsg msg;
  snprintf(msg.line, sizeof(msg.line), "%s", line.c_str());
  
  // Non-blocking send from Core 1 to Core 0 queue
  xQueueSend(sdQueue, &msg, 0);
}

void sdLogEvent(const char* msg) {
  if (!sdOk || !sdLoggingEnabled || msg == NULL) return;
  sdAppendLine(sdCsvLine("EVENT", String(msg)));
}

void sdLogEvent(const String& msg) {
  if (!sdOk || !sdLoggingEnabled) return;
  sdAppendLine(sdCsvLine("EVENT", msg));
}

void flushSdEventQueue() {
  // FreeRTOS queue handles buffered processing asynchronously on Core 0
}

void updateSdTelemetry() {
  if (!sdOk || !sdLoggingEnabled) return;

  uint32_t now = millis();
  if (now - lastSdTelemetryMs < SD_LOG_PERIOD_MS) return;
  lastSdTelemetryMs = now;

  sdAppendLine(sdCsvLine("DATA", ""));
}

void initSdLogging() {
  if (!sdLoggingEnabled) {
    Serial.println("SD LOG: disabled by config.");
    return;
  }

  if (sdMutex == NULL) {
    sdMutex = xSemaphoreCreateMutex();
  }

  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    bool mounted = mountSd();
    if (!mounted) {
      Serial.println("SD LOG: card not available. ECU continues without SD logging.");
      xSemaphoreGive(sdMutex);
      return;
    }

    sdWriteFailCount = 0;
    sdConsecFail = 0;

    bool slotsFull = false;
    for (uint16_t i = 0; i < 1000; i++) {
      snprintf(sdLogPath, sizeof(sdLogPath), "/ECU%03u.CSV", i);
      if (!SD.exists(sdLogPath)) break;
      if (i == 999) {
        slotsFull = true;
        Serial.println("SD LOG: WARNING - all 1000 slots full! Appending to ECU999.CSV.");
      }
    }
    xSemaphoreGive(sdMutex);

    if (sdQueue == NULL) {
      sdQueue = xQueueCreate(SD_QUEUE_SIZE, sizeof(SdLogMsg));
    }

    if (sdTaskHandle == NULL) {
      // Pin SD Logging task to Core 0 (xCoreID = 0)
      xTaskCreatePinnedToCore(
        sdTaskWorker,
        "sdTask",
        4096,
        NULL,
        1,            // Priority 1
        &sdTaskHandle,
        0             // Core 0
      );
    }

    if (!slotsFull) {
      sdAppendLine("ms,type,egtC,dEgtCps,rpm,fRpm,rRpm,pumpUs,starterUs,ign,valve1,valve2,event");
    }
    sdLogEvent(slotsFull ? "BOOT SD_LOG_READY (slots full)" : "BOOT SD_LOG_READY");

    Serial.print("SD LOG: OK (Async Core 0) file="); Serial.println(sdLogPath);
  }
}

void printSdStatus() {
  Serial.println("===== SD LOG STATUS =====");
  Serial.print("sdOk="); Serial.println(sdOk ? "OK" : "FAIL/NOT_INIT");
  Serial.print("sdLoggingEnabled="); Serial.println(sdLoggingEnabled ? "ON" : "OFF");
  Serial.print("sdLogPath="); Serial.println(sdLogPath);
  Serial.print("sdWriteFailCount="); Serial.println(sdWriteFailCount);
  Serial.print("sdQueueSpaces="); Serial.println(sdQueue ? uxQueueSpacesAvailable(sdQueue) : 0);
  Serial.println("Task: Core 0 (Async FreeRTOS)");
  Serial.println("Pins: CS=13 SCK=14 MOSI=23 MISO=27 (HSPI)");
  Serial.println("=========================");
}

// ---------------- Output apply ----------------
void applyOutputs() {
  pumpUs = constrain(pumpUs, ESC_MIN_US, ESC_MAX_US);
  startUs = constrain(startUs, ESC_MIN_US, ESC_MAX_US);
  static int lastPumpUs = -1, lastStartUs = -1;
  static int8_t lastIgn = -1, lastV1 = -1, lastV2 = -1;

  if (pumpUs != lastPumpUs) { escWriteUs(PIN_ESC_PUMP, LEDC_CH_PUMP, pumpUs, escPumpPeriodUs); lastPumpUs = pumpUs; }
  if (startUs != lastStartUs) { escWriteUs(PIN_ESC_START, LEDC_CH_START, startUs, escStartPeriodUs); lastStartUs = startUs; }
  
  int curIgn = (ignCmd == IGN_ACTIVE_HIGH) ? HIGH : LOW;
  int curV1  = (valve1Cmd == VALVE_ACTIVE_HIGH) ? HIGH : LOW;
  int curV2  = (valve2Cmd == VALVE_ACTIVE_HIGH) ? HIGH : LOW;

  if (curIgn != lastIgn) { digitalWrite(PIN_IGN, curIgn); lastIgn = curIgn; }
  if (curV1 != lastV1)   { digitalWrite(PIN_VALVE_1, curV1); lastV1 = curV1; }
  if (curV2 != lastV2)   { digitalWrite(PIN_VALVE_2, curV2); lastV2 = curV2; }
}

void emergencyStop() {
  setPumpTargetUs(ESC_SAFE_US);
  pumpUs = ESC_SAFE_US; // Ngắt lập tức không qua Ramp
  ignCmd = false; valve1Cmd = false; valve2Cmd = false;
  sdLogEvent("EV:A03_ESTOP");
  if (startTargetUs <= ESC_SAFE_US) {
    if (egt.ok && egt.c > 80.0f) {
      setStartTargetUs(1300);
      Serial.print("EV:A03|EGT="); Serial.println(egt.c, 1);
    } else {
      Serial.println("EV:A04");
    }
  } else {
    Serial.print("EV:A03|START="); Serial.println(startUs);
  }
  applyOutputs();
}

void allOff() {
  setPumpTargetUs(ESC_SAFE_US); setStartTargetUs(ESC_SAFE_US);
  pumpUs = ESC_SAFE_US; startUs = ESC_SAFE_US; // Ngắt lập tức không qua Ramp
  ignCmd = false; valve1Cmd = false; valve2Cmd = false;
  sdLogEvent("EV:A05_ALL_OFF");
  applyOutputs();
}

// ---------------- Purge & Prime Timers ----------------
// Removed per user request

bool escCalActive = false;
uint32_t escCalPhaseUntilMs = 0;
static const uint32_t ESC_CAL_MAX_HOLD_MS = 5000;

// ---------------- Serial console ----------------
long numberAfter(const String& cmd, const String& prefix) { return cmd.substring(prefix.length()).toInt(); }
float numberAfterFloat(const String& cmd, const String& prefix) { return cmd.substring(prefix.length()).toFloat(); }

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
  Serial.println("estop                       -> emergency stop (fuel/ign off, starter purges)");
  Serial.println("alloff | stop               -> all outputs to safe (total shutdown)");
  Serial.println("savecfg | loadcfg           -> save/reload sensor setup to SD");
  Serial.println("sdstatus | sdtest           -> check SD status / write test mark");
  Serial.println("set sdlog on|off            -> enable/disable SD telemetry logging");
}

void sendWebStatus() {
  Serial.print("WEB_DATA|");
  Serial.print("EGT=");
  if (egt.ok) { Serial.print(egt.c, 1); Serial.print("C"); } else { Serial.print("ERR("); Serial.print(egtFaultString(egt.fault)); Serial.print(")"); }
  Serial.print(" | dEGT="); Serial.print(egt.gradientCps, 1);
  Serial.print(" | PEGT="); if (!isnan(egt.cProjected3s)) { Serial.print(egt.cProjected3s, 1); } else { Serial.print("-"); }
  Serial.print(" | RPM="); Serial.print(rpmData.rpm, 0);
  Serial.print(" | FRPM="); Serial.print(rpmData.rpmFiltered, 0);
  Serial.print(" | RRPM="); Serial.print(rpmData.rawRawRpm, 0);
  Serial.print(" | SIG="); Serial.print(rpmData.signalRecent ? "OK" : (rpmData.noise == RPM_REST_NOISE ? "REST" : "LOST"));
  Serial.print(" | RNOISE="); Serial.print(rpmNoiseName(rpmData.noise));
  Serial.print(" | PUMP="); Serial.print(pumpUs); Serial.print("us");
  Serial.print(" | START="); Serial.print(startUs); Serial.print("us");
  Serial.print(" | IGN="); Serial.print(ignCmd ? 1 : 0);
  Serial.print(" | V1="); Serial.print(valve1Cmd ? 1 : 0);
  Serial.print(" | V2="); Serial.print(valve2Cmd ? 1 : 0);
  Serial.print(" | SD="); Serial.print(sdOk ? "OK" : "-");
  if (rpmCalMode) Serial.print(" | RPMCAL");
  Serial.print(" | ALEARN="); Serial.print(starterLearnedBins); Serial.print("/"); Serial.print(PWM_BIN_COUNT);
  Serial.print(" | PRAMP="); Serial.print(pumpRampEnabled ? 1 : 0);
  Serial.print(" | SRAMP="); Serial.print(starterRampEnabled ? 1 : 0);
  if (otaUpdatePending) {
    int remainSec = 30 - ((millis() - otaNoticeStartMs) / 1000);
    if (remainSec < 0) remainSec = 0;
    Serial.print(" | OTA=PENDING_"); Serial.print(remainSec); Serial.print("S");
  } else {
    Serial.print(" | OTA="); Serial.print(otaEnabled ? (otaInProgress ? "BUSY" : "READY") : "OFF");
  }
  Serial.print(" | WIFI=");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(WiFi.localIP());
  } else if (wifiConnecting) {
    Serial.print("CONNECTING");
  } else {
    Serial.print("OFF");
  }
  Serial.print(" | VER=7.9");
  Serial.println();
}

void sendWebConfig() {
  Serial.print("CONFIG_DATA|");
  Serial.print("START_SET="); Serial.print(startSetUs);
  Serial.print(" | PUMP_SET="); Serial.print(pumpSetUs);
  Serial.print(" | START_STEP="); Serial.print(startStepUs);
  Serial.print(" | PUMP_STEP="); Serial.print(pumpStepUs);
  Serial.print(" | PEGT_MAX="); Serial.print(pegtLimit, 1);
  Serial.print(" | EGT_MAX="); Serial.print(egtMaxLimit, 1);
  Serial.print(" | MAX_RPM="); Serial.print(maxRpmLimit, 0);
  Serial.print(" | WIFI_SSID="); Serial.print(wifiSsid);
  Serial.print(" | OTA_URL="); Serial.print(otaUrl);
  Serial.println();
}
void printStatus() {
  Serial.print("EGT=");
  if (egt.ok) { Serial.print(egt.c, 1); Serial.print("C"); } else { Serial.print("ERR("); Serial.print(egtFaultString(egt.fault)); Serial.print(")"); }
  Serial.print(" | dEGT="); Serial.print(egt.gradientCps, 1);
  Serial.print(" | RPM="); Serial.print(rpmData.rpm, 0);
  Serial.print(" | FRPM="); Serial.print(rpmData.rpmFiltered, 0);
  Serial.print(" | RRPM="); Serial.print(rpmData.rawRawRpm, 0);
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
  Serial.print(" fRPM="); Serial.print(rpmData.rpmFiltered, 0);
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
  if (cmd == "rpmlearn reset") { resetLearnedRpmMap(); Serial.println("RPM learning table reset."); return; }
  if (cmd == "rpmlearn") { printLearnedRpmMap(); return; }

  if (cmd.startsWith("set rpmfilter ")) {
    int f = numberAfter(cmd, "set rpmfilter ");
    if (f < 20 || f > 5000) { Serial.println("ERROR: rpmfilter 20..5000 us"); return; }
    noInterrupts(); rpmMinPulseUs = (uint32_t)f; interrupts();
    resetRpmStats(); saveConfigToNvs(); Serial.print("rpmFilterUs="); Serial.println(f); return;
  }
  if (cmd.startsWith("set rpmedge ")) {
    String e = cmd.substring(String("set rpmedge ").length()); e.trim();
    if (e == "rising") rpmEdgeMode = RISING;
    else if (e == "falling") rpmEdgeMode = FALLING;
    else { Serial.println("ERROR: set rpmedge rising|falling"); return; }
    resetRpmStats(); attachRpmInterrupt(); saveConfigToNvs();
    Serial.print("rpmEdge="); Serial.println(rpmEdgeName()); return;
  }
  if (cmd.startsWith("set ppr ")) {
    int p = numberAfter(cmd, "set ppr ");
    if (p != 1 && p != 2) { Serial.println("ERROR: ppr 1 or 2"); return; }
    pulsesPerRev = p; resetRpmStats(); saveConfigToNvs(); Serial.println("OK"); return;
  }
  if (cmd.startsWith("set startset ")) {
    int v = numberAfter(cmd, "set startset ");
    startSetUs = clampInt(v, ESC_MIN_US, ESC_MAX_US);
    saveConfigToNvs(); Serial.print("startSetUs="); Serial.println(startSetUs); return;
  }
  if (cmd.startsWith("set pumpset ")) {
    int v = numberAfter(cmd, "set pumpset ");
    pumpSetUs = clampInt(v, ESC_MIN_US, ESC_MAX_US);
    saveConfigToNvs(); Serial.print("pumpSetUs="); Serial.println(pumpSetUs); return;
  }
  if (cmd.startsWith("set startstep ")) {
    int v = numberAfter(cmd, "set startstep ");
    startStepUs = clampInt(v, 1, 100);
    saveConfigToNvs(); Serial.print("startStepUs="); Serial.println(startStepUs); return;
  }
  if (cmd.startsWith("set pumpstep ")) {
    int v = numberAfter(cmd, "set pumpstep ");
    pumpStepUs = clampInt(v, 1, 100);
    saveConfigToNvs(); Serial.print("pumpStepUs="); Serial.println(pumpStepUs); return;
  }
  if (cmd.startsWith("set pegtmax ")) {
    float v = numberAfterFloat(cmd, "set pegtmax ");
    if (v < 300.0f || v > 1200.0f) { Serial.println("ERROR: pegtmax 300..1200 C"); return; }
    pegtLimit = v;
    saveConfigToNvs();
    Serial.print("pegtLimit="); Serial.println(pegtLimit, 1); sendWebConfig(); return;
  }
  if (cmd.startsWith("set egtmax ")) {
    float v = numberAfterFloat(cmd, "set egtmax ");
    if (v < 300.0f || v > 1200.0f) { Serial.println("ERROR: egtmax 300..1200 C"); return; }
    egtMaxLimit = v;
    saveConfigToNvs();
    Serial.print("egtMaxLimit="); Serial.println(egtMaxLimit, 1); sendWebConfig(); return;
  }
  if (cmd.startsWith("set maxrpm ")) {
    float v = numberAfterFloat(cmd, "set maxrpm ");
    if (v < 5000.0f || v > 250000.0f) { Serial.println("ERROR: maxrpm 5000..250000 RPM"); return; }
    maxRpmLimit = v;
    saveConfigToNvs();
    Serial.print("maxRpmLimit="); Serial.println(maxRpmLimit, 0); sendWebConfig(); return;
  }
  if (cmd == "set rpmcal on") { rpmCalMode = true; resetRpmStats(); Serial.println("RPM CAL ON."); return; }
  if (cmd == "set rpmcal off") { rpmCalMode = false; resetRpmStats(); Serial.println("RPM CAL OFF."); return; }

  if (cmd == "set pumpramp on") { pumpRampEnabled = true; Serial.println("PUMP RAMP ON"); sendWebStatus(); return; }
  if (cmd == "set pumpramp off") { pumpRampEnabled = false; pumpUs = pumpTargetUs; Serial.println("PUMP RAMP OFF"); sendWebStatus(); return; }
  if (cmd == "set starterramp on") { starterRampEnabled = true; Serial.println("STARTER RAMP ON"); sendWebStatus(); return; }
  if (cmd == "set starterramp off") { starterRampEnabled = false; startUs = startTargetUs; Serial.println("STARTER RAMP OFF"); sendWebStatus(); return; }

  if (cmd == "sdstatus") { printSdStatus(); return; }
  if (cmd == "sdtest") { sdLogEvent("SD_TEST_MARK"); flushSdEventQueue(); Serial.println("SD test mark logged."); return; }
  if (cmd == "set sdlog on")  { sdLoggingEnabled = true;  sdLogEvent("SD_LOG_ON");  Serial.println("SD log ON");  return; }
  if (cmd == "set sdlog off") { sdLogEvent("SD_LOG_OFF"); sdLoggingEnabled = false; Serial.println("SD log OFF"); return; }

  if (cmd.startsWith("set wifi ")) {
    String args = cmd.substring(String("set wifi ").length()); args.trim();
    if (args == "off" || args == "disconnect") {
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      wifiConnecting = false;
      wifiSsid[0] = '\0'; wifiPass[0] = '\0';
      saveConfigToNvs();
      Serial.println("WIFI: Disconnected & saved.");
      sendWebStatus();
      return;
    }
    int spaceIdx = args.indexOf(' ');
    if (spaceIdx > 0) {
      String ssid = args.substring(0, spaceIdx); ssid.trim();
      String pass = args.substring(spaceIdx + 1); pass.trim();
      ssid.toCharArray(wifiSsid, sizeof(wifiSsid));
      pass.toCharArray(wifiPass, sizeof(wifiPass));
      saveConfigToNvs();
      connectWifi();
      sendWebStatus();
      return;
    } else {
      String ssid = args; ssid.trim();
      ssid.toCharArray(wifiSsid, sizeof(wifiSsid));
      wifiPass[0] = '\0';
      saveConfigToNvs();
      connectWifi();
      sendWebStatus();
      return;
    }
  }
  if (cmd == "wifi status") {
    Serial.print("WIFI: ");
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("CONNECTED | IP="); Serial.print(WiFi.localIP());
      Serial.print(" | RSSI="); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
    } else {
      Serial.println(wifiConnecting ? "CONNECTING..." : "DISCONNECTED");
    }
    return;
  }
  if (cmd == "wifi scan" || cmd == "scan wifi") {
    Serial.println("WIFI: Starting async scan...");
    WiFi.mode(WIFI_STA);
    WiFi.scanNetworks(true);
    wifiScanRequested = true;
    wifiScanStartMs = millis();
    return;
  }

  if (cmd == "ota check" || cmd == "set ota check" || cmd == "check ota") {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("HTTP OTA: Cannot check - WiFi not connected.");
      return;
    }
    Serial.println("HTTP OTA: Manual check requested by user...");
    checkHttpOtaOnBoot();
    return;
  }
  if (cmd == "ota confirm" || cmd == "set ota confirm") {
    if (!otaUpdatePending) {
      Serial.println("HTTP OTA: No update pending to confirm.");
      return;
    }
    otaUpdatePending = false;
    Serial.println("HTTP OTA: User CONFIRMED! Starting update...");
    performHttpOtaUpdate();
    return;
  }
  if (cmd == "ota cancel" || cmd == "set ota cancel") {
    if (otaUpdatePending) {
      otaUpdatePending = false;
      Serial.println("HTTP OTA: CANCELED by user.");
      Serial.println("EV:OTA_CANCELED");
      sendWebStatus();
    } else {
      Serial.println("HTTP OTA: No update pending.");
    }
    return;
  }
  if (cmd == "set ota on") {
    if (WiFi.status() == WL_CONNECTED) {
      initOtaCore0();
      Serial.println("OTA: Enabled on Core 0.");
    } else {
      Serial.println("OTA: Cannot enable - WiFi not connected.");
    }
    return;
  }
  if (cmd == "set ota off") {
    otaEnabled = false;
    Serial.println("OTA: Disabled.");
    return;
  }
  if (cmd.startsWith("set ota host ") || cmd.startsWith("set ota url ")) {
    String h;
    if (cmd.startsWith("set ota host ")) h = cmd.substring(String("set ota host ").length());
    else h = cmd.substring(String("set ota url ").length());
    h.trim();
    if (h.length() > 0) {
      h.toCharArray(otaUrl, sizeof(otaUrl));
      initOtaCore0();
      saveConfigToNvs();
      Serial.println("HTTP OTA: Host URL set to -> " + String(otaUrl));
      sendWebConfig();
    }
    return;
  }

  if (cmd.startsWith("starter ")) {
    String arg = cmd.substring(String("starter ").length()); arg.trim();
    if (arg == "off") { setStartTargetUs(ESC_SAFE_US); applyOutputs(); sdLogEvent("STARTER OFF"); Serial.println("STARTER OFF"); return; }
    int us = arg.toInt();
    if (us < ESC_MIN_US || us > ESC_MAX_US) { Serial.println("ERROR: starter 1000..2000"); return; }
    setStartTargetUs(us); applyOutputs();
    sdLogEvent(String("STARTER ") + us);
    Serial.print("STARTER="); Serial.println(us); return;
  }
  if (cmd.startsWith("pump ")) {
    String arg = cmd.substring(String("pump ").length()); arg.trim();
    if (arg == "off") {
      setPumpTargetUs(ESC_SAFE_US);
      pumpUs = ESC_SAFE_US; // Ngắt Bơm ngay lập tức
      bool wasValveOpen = valve1Cmd || valve2Cmd;
      valve1Cmd = false; valve2Cmd = false;
      applyOutputs();
      sdLogEvent("PUMP OFF");
      if (wasValveOpen) Serial.println("EV:A01");
      else Serial.println("PUMP OFF");
      sendWebStatus();
      return;
    }
    int us = arg.toInt();
    if (us < ESC_MIN_US || us > ESC_MAX_US) { Serial.println("ERR:PUMP_RANGE"); return; }
    
    // KHÓA AN TOÀN: Không cho bật Bơm nếu cả 2 van nhiên liệu đang đóng
    if (us > ESC_SAFE_US && !valve1Cmd && !valve2Cmd) {
      Serial.println("ERR:E01");
      sendWebStatus();
      return;
    }

    setPumpTargetUs(us);
    if (pumpTargetUs <= ESC_SAFE_US) {
      pumpUs = ESC_SAFE_US;
      bool wasValveOpen = valve1Cmd || valve2Cmd;
      valve1Cmd = false; valve2Cmd = false;
      if (wasValveOpen) Serial.println("EV:A01");
    }
    applyOutputs();
    sdLogEvent(String("PUMP ") + us);
    Serial.print("PUMP="); Serial.println(us); return;
  }
  if (cmd == "ign on")  { ignCmd = true;  applyOutputs(); sdLogEvent("IGN ON");  Serial.println("IGN ON");  return; }
  if (cmd == "ign off") { ignCmd = false; applyOutputs(); sdLogEvent("IGN OFF"); Serial.println("IGN OFF"); return; }
  if (cmd == "valve1 on")  { valve1Cmd = true;  applyOutputs(); sdLogEvent("VALVE1 ON");  Serial.println("VALVE1 ON");  return; }
  if (cmd == "valve1 off") { 
    valve1Cmd = false; 
    sdLogEvent("VALVE1 OFF");
    if (!valve1Cmd && !valve2Cmd && pumpTargetUs > ESC_SAFE_US) {
      setPumpTargetUs(ESC_SAFE_US);
      pumpUs = ESC_SAFE_US;
      Serial.println("EV:A02");
    } else {
      Serial.println("VALVE1 OFF"); 
    }
    applyOutputs(); sendWebStatus(); return; 
  }
  if (cmd == "valve2 on")  { valve2Cmd = true;  applyOutputs(); sdLogEvent("VALVE2 ON");  Serial.println("VALVE2 ON");  return; }
  if (cmd == "valve2 off") { 
    valve2Cmd = false; 
    sdLogEvent("VALVE2 OFF");
    if (!valve1Cmd && !valve2Cmd && pumpTargetUs > ESC_SAFE_US) {
      setPumpTargetUs(ESC_SAFE_US);
      pumpUs = ESC_SAFE_US;
      Serial.println("EV:A02");
    } else {
      Serial.println("VALVE2 OFF"); 
    }
    applyOutputs(); sendWebStatus(); return; 
  }

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

  if (cmd == "estop" || cmd == "coolstop") {
    emergencyStop(); Serial.println("EV:A03"); return;
  }
  if (cmd == "alloff" || cmd == "stop" || cmd == "off" || cmd == "shutdown") { 
    allOff(); Serial.println("EV:A05"); return; 
  }
  if (cmd == "savecfg") { 
    if (saveConfigToSd()) {
      sendWebStatus();
    }
    return; 
  }
  if (cmd == "loadcfg") { 
    if (loadConfigFromSd()) { 
      resetRpmStats(); 
      attachRpmInterrupt(); 
      applyOutputs();      // Bắt buộc ESC chạy theo mức PWM mới load
      Serial.println("Config reloaded."); 
      sendWebConfig();     // Gửi CONFIG_DATA để cập nhật Text Box
      sendWebStatus();     // Gửi WEB_DATA để UI đồng bộ thanh trượt
    } else {
      Serial.println("Config reload failed!");
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

  Serial.begin(115200);
  delay(400);

  pinMode(PIN_IGN, OUTPUT);     digitalWrite(PIN_IGN, IGN_ACTIVE_HIGH ? LOW : HIGH);
  pinMode(PIN_VALVE_1, OUTPUT); digitalWrite(PIN_VALVE_1, VALVE_ACTIVE_HIGH ? LOW : HIGH);
  pinMode(PIN_VALVE_2, OUTPUT); digitalWrite(PIN_VALVE_2, VALVE_ACTIVE_HIGH ? LOW : HIGH);
  
  pinMode(PIN_LED, OUTPUT);     digitalWrite(PIN_LED, LOW); // Heartbeat LED
  pinMode(PIN_BUTTON, INPUT_PULLUP);                        // User Button (Active LOW)
  pinMode(PIN_RPM, INPUT);

  gpio_install_isr_service(0); // Cài đặt ISR service trước để tránh lỗi khi gọi attachInterrupt nhiều lần

  mountSd();
  loadConfigFromNvs(); // Load ESP32 Flash Preferences NVS first
  loadConfigFromSd();  // Load SD config override if SD card is present
  initSdLogging();
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

  Serial.println("ECU Manual V1 booted (Serial-only, fully manual) - VERSION 7.9");
  Serial.print("MAX31855 begin() = "); Serial.println(thermoOk ? "OK" : "CHECK_WIRING");

  if (strlen(wifiSsid) > 0) {
    connectWifi();
  }
}
unsigned long lastStatusTime = 0;
unsigned long lastHeartbeatMs = 0;
bool ledState = false;

static unsigned long btnPressStartMs = 0;
static bool btnStateLast = HIGH;
static bool longPressTriggered = false;

void updateButton() {
  bool curState = digitalRead(PIN_BUTTON);
  uint32_t now = millis();

  if (curState == LOW && btnStateLast == HIGH) {
    btnPressStartMs = now;
    longPressTriggered = false;
  }

  if (curState == LOW) {
    if (!longPressTriggered && (now - btnPressStartMs >= 3000)) {
      longPressTriggered = true;
      allOff();
      Serial.println("EV:A05");
      sendWebStatus();
    }
  }

  if (curState == HIGH && btnStateLast == LOW) {
    uint32_t duration = now - btnPressStartMs;
    if (duration >= 50 && duration < 3000 && !longPressTriggered) {
      if (otaUpdatePending) {
        otaUpdatePending = false;
        Serial.println("-> [BUTTON SHORT PRESS]: OTA Update CONFIRMED via physical button!");
        performHttpOtaUpdate();
      } else {
        emergencyStop();
        Serial.println("-> [BUTTON SHORT PRESS]: EMERGENCY PURGE STOP (Starter ON, Fuel/Ign OFF).");
        sendWebStatus();
      }
    }
  }

  btnStateLast = curState;
}

void loop() {
  processSerialRx();
  updateButton();

  if (wifiScanRequested) {
    int scanRes = WiFi.scanComplete();
    if (scanRes >= 0) {
      wifiScanRequested = false;
      if (scanRes == 0) {
        Serial.println("WIFISCAN: NONE");
      } else {
        Serial.print("WIFISCAN: ");
        for (int i = 0; i < scanRes; ++i) {
          if (i > 0) Serial.print("|");
          Serial.print(WiFi.SSID(i));
          Serial.print(",");
          Serial.print(WiFi.RSSI(i));
          Serial.print(",");
          Serial.print(WiFi.encryptionType(i) == 0 ? "OPEN" : "SEC");
        }
        Serial.println();
      }
      WiFi.scanDelete();
    } else if (millis() - wifiScanStartMs > 10000) {
      wifiScanRequested = false;
      Serial.println("WIFISCAN: NONE");
      WiFi.scanDelete();
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (wifiConnecting) {
      wifiConnecting = false;
      Serial.print("WIFI: Connected! Local IP: ");
      Serial.println(WiFi.localIP());
      sendWebStatus();
    }
    if (!otaEnabled) {
      initOtaCore0();
    }
    if (!otaBootCheckDone) {
      otaBootCheckDone = true;
      checkHttpOtaOnBoot();
    }
  }

  updateOtaNotice();

  if (escCalActive && escCalPhaseUntilMs > 0 && millis() >= escCalPhaseUntilMs) {
    setStartTargetUs(ESC_MIN_US); setPumpTargetUs(ESC_MIN_US);
    startUs = ESC_MIN_US; pumpUs = ESC_MIN_US; applyOutputs();
    escCalPhaseUntilMs = 0; escCalActive = false;
    Serial.println("ESC CAL: dropped to MIN 1000us (done).");
  }

  updateRpm();
  updateEgt();
  updateRamps(); // Tính toán xung PWM mượt theo S-Curve & Exponential
  applyOutputs(); 
  updateSdTelemetry();
  flushSdEventQueue(); 

  // --- Heartbeat LED ---
  uint32_t ledPeriod = otaUpdatePending ? 100 : 500; // Blink fast (100ms) when waiting for OTA confirmation
  if (millis() - lastHeartbeatMs >= ledPeriod) {
    lastHeartbeatMs = millis();
    ledState = !ledState;
    digitalWrite(PIN_LED, ledState ? HIGH : LOW);
  }

  if (millis() - lastStatusPrintMs >= STATUS_PRINT_MS) {
    lastStatusPrintMs = millis();
    if (rpmDetailMode) printRpmDetail();
  }
  if (millis() - lastStatusTime > 500) {
    lastStatusTime = millis();
    sendWebStatus(); 
  }
}