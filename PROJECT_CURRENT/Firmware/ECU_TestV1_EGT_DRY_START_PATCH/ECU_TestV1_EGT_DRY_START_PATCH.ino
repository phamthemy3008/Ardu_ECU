/*
  ESP32 Test ECU V1 - Serial Preview Firmware
  Pins: MAX31855 CLK=18 CS=5 DO=19, RPM=33, PUMP=26, START=25,
        VALVE1=17, VALVE2=16, IGN/GLOW=32.
  Valve roles (per EnJet E86/G3 manual "Component Test" section):
    VALVE1 = Start solenoid valve (start oil circuit) - open only while
             MODE_STARTING (ignition dosing through accel-to-idle), closes
             automatically once the engine reaches MODE_IDLING.
    VALVE2 = Main oil valve (main oil circuit) - open whenever fuel is
             commanded, same as a standalone bench "Oil Pump Test".
  Web UI + Serial Monitor. Auto-start is disabled at boot.
  User button: GPIO22 active-low, short press=status/log, hold=ARM/START/CLEAR, running press=soft stop.
  Status LED: onboard GPIO2 active-low.
  SD logging: light CSV log on SPI microSD module, CS=13 SCK=14 MOSI=23 MISO=27.

  Rev noise-diagnostics + RPM guard:
  - RPM ISR records accepted count, raw edges, rejected/glitch edges.
  - REST RPM guard rejects isolated pulses while WAITING and all outputs are OFF.
  - Commands: rpmdetail, rpmdetail on/off, set rpmfilter <us>, set rpmedge rising|falling, rpmreset.

  Rev hybrid-fuel-control:
  - Keeps PumpPoint calibration table for us <-> ml/min estimate and hard pump limits.
  - Adds Ardu_ECU-style fuelTargetUs/fuelNow control: target RPM decides whether to nudge fuel target up/down.
  - pumpUs still ramps one microsecond at a time with accel/decel delays; no direct jump to target.

  Rev WebUI + Test Wizard:
  - ESP32 SoftAP: ECU_TestV1 / admin1234, http://192.168.4.1
  - Dashboard, controls, Test Wizard, checklist interlock, and event log.

  Rev safety patch:
  - Adds ACCEL_TO_IDLE timeout to avoid holding fuel/starter indefinitely.
  - Adds real cooldown airflow using starter after soft stop/abort. Fuel, valves and igniter stay OFF during cooldown.

  Rev EGT-open dry-start patch:
  - If thermocouple/MAX31855 is OPEN, startidle is no longer hard-blocked by EGT.
  - In that case startidle automatically becomes DRY START/RPM TEST: starter runs, but PUMP, VALVES and IGN stay OFF.
  - Real fuel start still requires valid EGT. This avoids blind fueling with no temperature feedback.
  - Serial pumptest now has an auto-timeout to avoid accidentally leaving the pump running.
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
#include <WiFi.h>
#include <WebServer.h>

// Forward declarations of the state enums (full definitions are further down).
// Required because escAttach()/escWriteUs() below are now the first functions in
// the sketch, so the Arduino IDE injects its auto-generated function prototypes
// right here - ahead of the real enum definitions. Any auto-prototype that names
// one of these enums (modeName(EcuMode), enterStage(StartStage), etc.) would
// otherwise fail to compile with "'EcuMode' was not declared in this scope".
// A fixed underlying type makes these opaque forward declarations legal (C++11).
enum EcuMode : uint8_t;
enum StartStage : uint8_t;
enum RpmNoiseLevel : uint8_t;
enum TestId : int8_t;
enum TestResult : uint8_t;

// ESC PWM is driven directly through the ESP32 LEDC peripheral instead of the
// ESP32Servo library. ESP32Servo's internal timer/LEDC setup has been observed
// to produce an unstable pulse on some ESP32 Arduino core versions (core 3.x
// changed the LEDC API), which an ESC reads as noisy/invalid throttle and
// responds to by stuttering/cutting the motor - independent of supply current.
// Raw LEDC removes that dependency and matches the exact API of the installed
// core.
#define PIN_EGT_CLK    18
#define PIN_EGT_CS      5
#define PIN_EGT_DO     19
#define PIN_RPM        33
#define PIN_ESC_PUMP   26
#define PIN_ESC_START  25
#define PIN_VALVE_1    17   // Start solenoid valve (start oil circuit) - MODE_STARTING only
#define PIN_VALVE_2    16   // Main oil valve (main oil circuit) - open whenever fuel is commanded
#define PIN_IGN        32
#define PIN_USER_BTN   22   // USER/ARM/START/SOFT-STOP button, active LOW
#define PIN_STATUS_LED  2    // onboard NodeMCU-32S LED, active LOW on tested board
#define PIN_SD_CS      13   // SPI microSD CS
#define PIN_SD_SCK     14   // SPI microSD SCK/CLK
#define PIN_SD_MOSI    23   // SPI microSD MOSI/DI
#define PIN_SD_MISO    27   // SPI microSD MISO/DO
// Input-only, ADC1 (no WiFi/ADC2 conflict), unused elsewhere on this board. Used
// only by "esctest" (see handleCommand()): jumper a wire from this pin to
// PIN_ESC_PUMP/PIN_ESC_START to measure the ESP32's own actual output pulse width
// with pulseIn(), when a scope isn't available/practical for a 50Hz signal.
#define PIN_ESC_SELFTEST_IN 34

static const bool IGN_ACTIVE_HIGH = true;
static const bool VALVE_ACTIVE_HIGH = true;

static const int ESC_SAFE_US = 1000;
static const int ESC_MIN_US  = 1000;
static const int ESC_MAX_US  = 2000;

// ---- ESC PWM via raw LEDC (see include-block comment above) ----
static const int ESC_PWM_FREQ_HZ = 50;
static const int ESC_PWM_RES_BITS = 16;
// legacyChannel is only read on core <3 (ledcWrite(channel, ...)); on core >=3
// ledcWrite() takes the pin directly, so these are unused there but still
// need to exist so the call sites below compile under both cores.
static const int LEDC_CH_PUMP  = 0;
static const int LEDC_CH_START = 1;

// escWriteUs() used to assume an exact 20000us (50Hz) period when converting us ->
// duty. LEDC can only hit a frequency the clock divider allows - at 16-bit
// resolution the real output is often a few tens of Hz off nominal 50Hz - so the
// actual pulse width was silently a bit off from the requested us. That slack was
// invisible while the ESC's learned MIN/MAX endpoints were far apart, but after a
// BLHeliSuite throttle-range recalibration to tighter endpoints it was enough to
// push the pulse outside the ESC's accepted range, so the ESC stopped responding
// even though a real PWM signal was still present on the pin (confirmed on scope).
// Fix: read back the frequency LEDC actually configured via ledcReadFreq() right
// after attach, and use that real period for every duty conversion instead of the
// assumed 20000us. Cached per-output since pump/starter may land on slightly
// different actual frequencies depending on which LEDC timer they get assigned.
double escPumpPeriodUs = 20000.0;
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
static const uint32_t RPM_SAMPLE_MS = 100;
static const uint32_t RPM_SIGNAL_TIMEOUT_MS = 1000;
// RPM guard: while ECU is WAITING and every output is OFF, any isolated RPM edge is treated as noise.
// This prevents 1-3 fake pulses from being interpreted as 250..1500 RPM.
static const uint32_t RPM_REST_GUARD_MAX_US = ESC_SAFE_US + 5;
static const uint32_t EGT_READ_PERIOD_MS = 120;
static const uint32_t EGT_STUCK_WARN_MS = 6000;  // warn (log only) if EGT reads frozen this long while combusting
static const uint32_t STATUS_PRINT_MS = 250;
static const uint32_t STAGE2_ARM_TIME_MS = 10000;
static const uint32_t STARTER_PROVE_TIMEOUT_MS = 1500;
// Test Wizard starter spin duration. 3s was too short: from a standing start the
// motor is still accelerating at the end of the window, so per-window jitter stays
// high and the RPM verdict false-flags NOISY before the speed has settled. 5s lets
// the motor reach a steady RPM so the final sample window reads CLEAN (matches the
// bench TEST_STARTER.ino observation that a fixed PWM settles within ~1-1.5s/step
// but a full spin-up from OFF takes noticeably longer).
static const uint32_t STARTER_TEST_SPIN_MS = 5000;
// Faster RPM-loss detection while fueled at running RPM (IDLING/OPERATING), so fuel
// is cut promptly after a real flameout instead of waiting the full 1s signal timeout.
static const uint32_t FUELED_RPM_LOSS_TIMEOUT_MS = 400;
// While running (IDLING/OPERATING), pulses are still arriving but the signal is
// classified NOISY: require it to persist this long before RPM_SIGNAL_LOST, so a
// single transient EMI burst (one ~100ms window) can't false-abort a healthy
// engine. A genuine loss of pulses is still caught by the recency check above.
static const uint32_t RPM_NOISY_ABORT_MS = 300;
// Short EGT look-ahead used only by the OVER_TEMP hard abort, to compensate for the
// ~1-sample (EGT_READ_PERIOD_MS) staleness of egt.c so a fast rise cannot overshoot
// maxEgtC by ~one sample before the abort reacts. Deliberately small (not the 3s
// fuel-control look-ahead) so it does not false-trip on normal light-off gradients.
static const float EGT_ABORT_LOOKAHEAD_S = 0.2f;
static const uint32_t SD_LOG_PERIOD_MS = 500;       // light telemetry logging: 2 lines/sec while active
static const uint32_t SD_SPI_HZ = 1000000;          // tested OK after FAT32 format; lower to 400000 if needed

// GPIO22 USER button logic, active-low.
// WAITING: short=status, hold 2s=ARM. ARMED: hold 3s=START IDLE.
// STARTING/IDLING/OPERATING: any press=SOFT STOP. ABORTED: hold 2s=clear if EGT safe.
static const bool USER_BTN_ACTIVE_LOW = true;
static const bool STATUS_LED_ACTIVE_LOW = true;  // tested: LOW = LED ON, HIGH = LED OFF
static const uint32_t BTN_DEBOUNCE_MS = 35;
static const uint32_t BTN_ARM_HOLD_MS = 2000;
static const uint32_t BTN_START_HOLD_MS = 3000;
static const uint32_t BTN_CLEAR_ABORT_HOLD_MS = 2000;
// Software glitch filter for RPM input.
// 120us still allows about 500,000 RPM at 1 pulse/rev.
// Increase to 200..500us if glow/starter creates narrow false pulses.
volatile uint32_t rpmMinPulseUs = 120;
int rpmEdgeMode = RISING;

struct PumpPoint { int us; float mlMin; };
static const PumpPoint kPumpMap[] = {
  {1000, 0.0f}, {1160, 50.0f}, {1175, 80.0f}, {1250, 265.0f},
  {1260, 280.0f}, {1265, 360.0f}, {1270, 560.0f}, {1300, 600.0f}
};
static const size_t kPumpMapCount = sizeof(kPumpMap) / sizeof(kPumpMap[0]);

struct Config {
  bool autoStartEnabled = false;
  bool requireEgtForStart = true;
  // If EGT is OPEN during startidle, allow only a dry starter/RPM test.
  // PUMP, VALVES and IGN are forced OFF in dry mode. Real fuel start still requires EGT OK.
  bool allowDryStartWhenEgtFault = true;
  uint32_t dryStartRunMs = 5000;
  bool requireRpmForStart = true;
  bool abortOnEgtFault = true;
  bool abortOnRpmFault = true;
  uint8_t pulsesPerRev = 1;

  int ignitionThresholdC = 100;
  int maxEgtC = 680;
  int startTargetEgtC = 500;
  int maxTempGradientCps = 200;

  int maxRpm = 110000;
  int idleRpm = 42000;          // preview, cần tune theo engine thật
  int rpmTolerance = 5000;      // Ardu_ECU-style RPM deadband around target RPM
  int flameoutRpm = 15000;
  int starterReleaseRpm = 24000;
  int fuelConfirmRpm = 18000;
  int starterProveMinRpm = 500;

  // ---- Engine/starter protection (applied every loop in all modes) ----
  // (1) Hot soak-back guard: while EGT > cooldownTargetC (the "hot" threshold, 90C),
  //     keep RPM >= hotSpinMinRpm by running the starter at hotSpinUs, so a hot engine
  //     that has stopped/slowed does not let hot gas creep forward and damage the front.
  //     Below that EGT the starter may stop.
  // (2) Starter overspeed: above starterMaxRpm force the starter OFF (disengage) so the
  //     spun-up impeller cannot overload/over-spin the starter motor.
  int starterMaxRpm = 10000;   // starter forced OFF above this RPM (release impeller)
  int hotSpinMinRpm = 3000;    // minimum RPM held while hot
  int hotSpinUs     = 1200;    // starter PWM used to hold hotSpinMinRpm while hot

  uint32_t purgeTimeMs = 3000;         // PURGE dwell after reaching ignArmRpm
  uint32_t preheatMs = 2000;           // SPINUP_PREHEAT glow-on time (glow stays on after)
  uint32_t noIgnitionTimeoutMs = 6000; // LIGHTOFF: abort if EGT never reaches threshold
  uint32_t fuelConfirmTimeoutMs = 10000;
  uint32_t postIgnitionHeatMs = 2000;  // LIGHTOFF: wait after opening main valve before closing start valve
  uint32_t starterReleaseHoldMs = 1500;
  uint32_t idleStabilizeMs = 2000;

  // ---- Start sequence (PURGE -> SPINUP_PREHEAT -> INTRO_FUEL -> LIGHTOFF -> ACCEL_TO_IDLE) ----
  // PURGE: starter ramps from startRampFromUs, +1us per startRampStepMs (250 -> 4us/s),
  //   until RPM > ignArmRpm, then dwells purgeTimeMs to blow residue out.
  // SPINUP_PREHEAT: glow ON for preheatMs (stays ON).
  // INTRO_FUEL: pump at introFuelUs (min), open Start Valve (valve1); glow ON.
  // LIGHTOFF: wait fuelDelayMs for fuel; if EGT >= ignitionThresholdC open Main Valve
  //   (valve2), wait postIgnitionHeatMs, close Start Valve, wait flameProveMs; flame OK
  //   if EGT >= ignitionThresholdC and not dropped > flameOutDropC -> glow OFF, success.
  //   Fail on noIgnitionTimeoutMs or flame loss.
  // ACCEL_TO_IDLE: while flame alive but RPM not rising, bump pump +accelStepUs and wait
  //   accelHoldMs, up to idleRpm. Starter PWM tracks RPM linearly starterAssistUs ->
  //   startRampToUs until starterMaxRpm (release), then OFF.
  int      startRampFromUs   = 1150; // PURGE ramp start
  int      startRampToUs     = 1450; // starter-assist MAX PWM (also PURGE ramp cap)
  uint32_t startRampStepMs   = 250;  // ramp +1us per this many ms (250 -> 4us/s, i.e. 2us/0.5s)
  // PURGE PWM has 3 phases, in order (starterRampUs()):
  //  Phase 1: escArmHoldMs at ESC_SAFE_US (1000us, genuinely zero) so the ESC finishes its
  //     own arm sequence before any non-zero throttle is applied - jumping straight to
  //     startRampFromUs right after forceSafeOutputs() can leave the ESC only partially
  //     armed, which looks like "spins a couple turns then stalls/stutters".
  //  Phase 2: purgeStableMs held steady at startRampFromUs, so the starter settles into a
  //     stable spin before the PWM starts changing.
  //  Phase 3: ramp +1us per startRampStepMs, capped at startRampToUs.
  // (Applies to both dry test and a real start, since starterRampUs() is shared by both.)
  uint32_t escArmHoldMs      = 2000;  // ESC arm hold at ESC_SAFE_US before any throttle
  uint32_t purgeStableMs     = 10000; // steady-hold at startRampFromUs before ramping
  int      ignArmRpm         = 3000; // "spun up" RPM: purge done / start fuel above this
  uint32_t spinupRpmTimeoutMs = 57000; // abort if RPM never reaches ignArmRpm within this (includes escArmHoldMs + purgeStableMs)
  uint32_t fuelDelayMs       = 1000; // LIGHTOFF: wait for fuel to reach the chamber
  // Real light-off (not just glow heating the thermocouple past the threshold): require
  // EGT >= ignitionThresholdC AND EGT rising at >= lightOffMinRiseCps, held continuously
  // for lightOffConfirmMs, before accepting ignition.
  int      lightOffMinRiseCps = 15; // min dEGT/dt (C/s) to count as real combustion
  uint32_t lightOffConfirmMs  = 700; // rise condition must hold this long
  uint32_t flameProveMs      = 2000; // wait after closing start valve to confirm self-sustain
  int      flameOutDropC     = 10;   // flame-out if EGT drops > this within ~2s (or falls below ignitionThresholdC)
  uint32_t accelHoldMs       = 5000; // ACCEL: wait per fuel bump to observe RPM
  int      accelStepUs       = 1;    // ACCEL: pump us bump when RPM not rising
  uint32_t purgeOutMs        = 5000; // after a failed start, keep starter spinning this long to blow fuel out

  // Safety patch: do not stay in ACCEL_TO_IDLE forever.
  // If idle RPM is not reached within this window, fuel/ignition are cut and cooldown starts.
  uint32_t accelToIdleTimeoutMs = 20000;

  // Safety patch: real cooldown airflow after soft stop/abort.
  // Fuel, valves and igniter remain OFF while starter gently moves air through the engine.
  uint32_t cooldownMinMs = 5000;
  uint32_t cooldownTimeoutMs = 45000;
  int cooldownTargetC = 90;   // also the "engine hot" threshold for the soak-back guard
  int cooldownStarterUs = 1100;

  // Comm watchdog: while fueled (IDLING/OPERATING), the operator must prove the
  // Serial/Web link is alive (any command, or the Web UI's automatic /api poll)
  // within this window, or the ECU auto-aborts instead of running unattended
  // at the last commanded throttle forever.
  bool commWatchdogEnabled = true;
  uint32_t commTimeoutMs = 8000;

  // Fuel output ramp delays. Low-RPM delays are intentionally slower, similar to Ardu_ECU.
  uint32_t accelStepDelayMs = 100;
  uint32_t decelStepDelayMs = 120;
  uint32_t lowAccelStepDelayMs = 600;
  uint32_t lowDecelStepDelayMs = 500;

  // Hybrid fuel controller nudges target by us, then pumpUs follows target slowly.
  int fuelStepUs = 1;           // normal closed-loop correction step
  int fuelCutStepUs = 5;        // stronger cut for overtemp/overspeed

  int starterPurgeUs = 1100;
  int starterSpinUs = 1200;
  int starterAssistUs = 1200;

  int introFuelUs = 1160;       // ~50 ml/min - light-off dose, kept < idleFuelUs
  int idleFuelUs = 1175;        // ~80 ml/min preview
  int maxFuelUs = 1260;         // ~280 ml/min preview

  // Bench pump-prime test PWM (Test Wizard "Pump prime"). Decoupled from
  // introFuelUs so it can be tuned independently. Default is the LOWEST point
  // actually measured on the bench (Luu_Luong_Bom.txt: 1160us ~50 ml/min) -
  // 1210us was only an interpolated guess between calibration points, never
  // verified with a real flow measurement.
  int pumpTestUs = 1160;

  bool requireChecklistForStart = true; // require Test Wizard PASS before startidle
  bool webEnabled = true;               // SoftAP Web UI enabled by default
  bool sdLoggingEnabled = true;         // light CSV SD logging enabled by default
} cfg;

enum EcuMode : uint8_t { MODE_WAITING, MODE_STARTING, MODE_IDLING, MODE_OPERATING, MODE_COOLDOWN, MODE_ABORTED };
// Start stages:
//  PURGE          - starter ramps from 1150us (+1us/step) until RPM>ignArmRpm, then dwell
//  SPINUP_PREHEAT - glow ON preheatMs (stays on)
//  INTRO_FUEL     - pump min + open Start Valve (valve1), glow on
//  LIGHTOFF       - wait fuel, EGT>=100C -> open Main Valve, close Start Valve, prove flame
//  ACCEL_TO_IDLE  - bump fuel while RPM stalls, starter assist tracks RPM, release at 10k
enum StartStage : uint8_t { ST_NONE, ST_PURGE, ST_SPINUP_PREHEAT, ST_INTRO_FUEL, ST_LIGHTOFF, ST_ACCEL_TO_IDLE };

EcuMode ecuMode = MODE_WAITING;
StartStage startStage = ST_NONE;
uint32_t modeEnteredMs = 0, stageEnteredMs = 0, starterAboveReleaseSinceMs = 0, lastPumpStepMs = 0;
uint32_t startRampBeganMs = 0;    // starter-ramp anchor for the PURGE ramp
int      hotSpinCurUs = 0;        // current hot soak-back starter PWM (ramps 1200us +1us/0.1s); 0 = not spinning
uint32_t hotSpinLastStepMs = 0;   // last hot-spin ramp step time
uint8_t  lightoffPhase = 0;       // LIGHTOFF sub-phase: 0=wait fuel/EGT, 1=main-valve settle, 2=flame prove
uint32_t lightoffPhaseMs = 0;     // sub-phase start time
uint32_t lightoffRiseSinceMs = 0; // when the EGT>=thr & rising condition first held (0=not holding)
float    flameRefEgtC = 0.0f;     // rolling EGT reference (2s) to detect a flame-out drop
uint32_t flameRefMs = 0;          // time the flame reference was last snapshotted
uint32_t accelStepAtMs = 0;       // ACCEL: last fuel-bump observation time
float    accelLastRpm = 0.0f;     // ACCEL: RPM at last observation
uint32_t purgeOutUntilMs = 0;     // after a failed start: keep starter spinning to blow unburned fuel out until this time
uint32_t runningSinceMs = 0;  // first time stable IDLING was reached this run; NOT reset by IDLING<->OPERATING toggles (flameout grace anchor)
uint32_t rpmStatsResetAtMs = 0;  // last resetRpmStats(); RPM_SIGNAL_LOST is suppressed briefly after so a reset can't look like signal loss
uint32_t ignitionDetectedMs = 0;  // set when EGT crosses ignition threshold; anchors the post-ignition RPM-rise check
bool cooldownAfterAbort = false;
bool stage2Armed = false;
bool abortAcknowledged = false;  // must be set via clearabort before re-arm from ABORTED
bool runStartedByButton = false; // true if the current run was started by the physical button (comm watchdog does not apply)
uint32_t stage2ArmUntilMs = 0, manualIgnOffAtMs = 0, manualStartOffAtMs = 0;
uint32_t manualGlowRiseSinceMs = 0; // manual-page safety: same "real light-off" condition as ST_LIGHTOFF phase 0 (EGT>=ignitionThresholdC AND rising >= lightOffMinRiseCps), held since this time, while glow held on manually
// Manual page "AUTO START": mirrors ST_INTRO_FUEL/ST_LIGHTOFF exactly (valve1+pump+glow,
// then confirm real light-off, open valve2, close valve1, prove flame, glow off) but
// stays in WAITING/ABORTED - no starter/RPM involved, just the fuel/valve/glow sequencing.
bool     manAutoActive = false;
uint8_t  manAutoPhase = 0;        // mirrors lightoffPhase: 0=wait real light-off, 1=valve2 open/settle, 2=flame prove
uint32_t manAutoPhaseMs = 0;      // current sub-phase start time
uint32_t manAutoRiseSinceMs = 0;  // phase-0 rising-condition hold timer (mirrors lightoffRiseSinceMs)
uint32_t manAutoStartedMs = 0;    // overall sequence start time, for the no-ignition timeout
uint32_t lastOperatorLinkMs = 0;  // last Serial/Web command or Web UI /api poll; feeds the comm watchdog
String lastAbortReason = "NONE";
String serialCmdBuf = "";  // non-blocking serial command accumulator
int throttlePct = 0;

Adafruit_MAX31855 thermo(PIN_EGT_CLK, PIN_EGT_CS, PIN_EGT_DO);

void IRAM_ATTR rpmISR();
bool canStartAutoIdle(String& why);
bool egtSafeForAbortClear();
bool egtAllowsDeliberateAbortClear();
bool fuelCommandBlockedByHotEgt();
void beginAutoIdle();
void printStatus(bool force);
void handleCommand(String cmd);
String egtFaultString(uint8_t f);
void addLog(const String& msg);
bool checklistPassed(String& why);
bool dryStartRequested();
bool checklistPassedForAutoStart(String& why);
void updateDryStarting();
void printChecklist();
void resetChecklist();
void enterCooldown(bool afterAbort, const String& reason);
void runTestByName(const String& name);
void updateActiveTest();
void setupWebServer();
void startWebServer();
void stopWebServer();
void updateStatusLed();
void initSdLogging();
void updateSdTelemetry();
void sdLogEvent(const String& msg);
void flushSdEventQueue();
void printSdStatus();
String webStatusJson();

int pumpUs = ESC_SAFE_US, startUs = ESC_SAFE_US;
int fuelTargetUs = ESC_SAFE_US;   // Ardu_ECU-style target; pumpUs is the actual output now
int fuelTargetRpm = 0;            // current RPM target used by hybrid fuel controller
bool ignCmd = false, valve1Cmd = false, valve2Cmd = false;

// ESC throttle-range calibration ("esccal"): drives PUMP+STARTER ESCs to
// ESC_MAX_US so both learn the endpoint while the operator (re)powers them,
// then auto-drops to ESC_MIN_US after a hold window - mirrors the standard
// BLHeli manual throttle-calibration procedure, done from the ECU itself so
// no separate bench rig/receiver is needed. Non-blocking (millis()-driven)
// like the other manual-test auto-off timers below, not delay()-based.
bool escCalActive = false;
uint32_t escCalPhaseUntilMs = 0;
static const uint32_t ESC_CAL_MAX_HOLD_MS = 5000; // time held at MAX before auto-drop to MIN
bool dryStartActive = false;  // true only when EGT is OPEN and startidle is converted to dry starter/RPM test

struct EgtState {
  bool ok = false;
  float c = NAN, prevC = NAN, gradientCps = 0.0f;
  uint8_t fault = 0;
  uint32_t lastReadMs = 0, lastGoodMs = 0;
} egt;

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
volatile uint64_t isrSumDtSqUs = 0;  // sum of dt^2, used for stddev-based jitter (not swayed by a single outlier)

enum RpmNoiseLevel : uint8_t { RPM_CLEAN, RPM_WARN, RPM_NOISY, RPM_REST_NOISE, RPM_NO_SIGNAL };

struct RpmState {
  float rpm = 0.0f;
  float rpmWindow = 0.0f;
  float rpmPeriod = 0.0f;
  float avgIntervalUs = 0.0f;
  float jitterPct = 0.0f;
  float rejectPct = 0.0f;
  float rpmDiffPct = 0.0f;
  bool signalRecent = false;
  bool restGuardActive = false;
  bool restPulseNoise = false;
  uint32_t acceptedWindow = 0;
  uint32_t rawEdges = 0;
  uint32_t rejectedEdges = 0;
  uint32_t validIntervals = 0;
  uint32_t lastPeriodUs = 0;
  uint32_t minIntervalUs = 0;
  uint32_t maxIntervalUs = 0;
  uint32_t filterUs = 0;
  uint32_t lastComputedMs = 0;
  uint32_t lastComputedUs = 0;
  uint32_t lastAcceptedPulseUsSnapshot = 0;
  int pinLevel = 0;
  RpmNoiseLevel noise = RPM_NO_SIGNAL;
} rpmData;
uint32_t lastStatusPrintMs = 0;
bool rpmDetailMode = false;   // OFF by default: basic status line only
// OFF by default: the rest-guard (rpmAtRestGuardCondition()) must stay active so a
// stationary sensor cannot show phantom RPM from ambient noise. Turn this ON only
// for a deliberate bench calibration session (spinning the sensor by hand while
// WAITING) via "set rpmcal on", then back OFF with "set rpmcal off" when done.
bool rpmCalMode = false;

// ===== Web UI =====
// SECURITY: anyone who joins this SoftAP can arm and start the real engine via
// /cmd (there is no per-command auth). CHANGE WEB_PASS to a strong, private value
// before any live-engine use, and only power the AP up when you intend to operate.
// The default below is a weak placeholder for bench bring-up only.
static const char* WEB_SSID = "ECU_TestV1";
static const char* WEB_PASS = "admin1234";   // <-- CHANGE ME (min 8 chars) before real runs
IPAddress webIp(192, 168, 4, 1);
IPAddress webGateway(192, 168, 4, 1);
IPAddress webSubnet(255, 255, 255, 0);
WebServer server(80);
bool webRoutesReady = false;
bool webStarted = false;

// ===== SD logging =====
SPIClass sdSPI(VSPI);
bool sdOk = false;
bool sdMounted = false;                 // SD.begin() has been attempted (mount is idempotent)
static const char* CONFIG_PATH = "/ECUCFG.TXT";
bool cfgLoadedFromSd = false;           // true once a saved config file was applied at boot
bool cfgFileOnSd = false;               // cached: a /ECUCFG.TXT exists (updated on save/load, not polled)
char sdLogPath[24] = "/ECU000.CSV";
uint32_t lastSdTelemetryMs = 0;
uint32_t sdWriteFailCount = 0;

// Deferred SD event queue: event lines are snapshotted at event time but the blocking
// SD.open()/close() is done from the loop AFTER the safety checks, so a slow card can
// never delay checkFailures()/applyOutputs() during a fast start sequence.
static const uint8_t SD_EVENT_QUEUE = 8;
String sdEventQueue[SD_EVENT_QUEUE];
uint8_t sdEventQHead = 0, sdEventQTail = 0;
uint32_t sdEventDropCount = 0;

// ===== Event log =====
static const uint8_t LOG_COUNT = 16;
String eventLog[LOG_COUNT];
uint8_t eventLogHead = 0;

// ===== Test Wizard / Checklist =====
enum TestId : int8_t {
  TEST_EGT = 0,
  TEST_RPM_NOISE,
  TEST_IGN,
  TEST_STARTER,
  TEST_STARTER_IGN,
  TEST_VALVE1,
  TEST_VALVE2,
  TEST_PUMP,
  TEST_KILL,
  TEST_COUNT,
  TEST_NONE = -1
};

enum TestResult : uint8_t { TEST_NOT_RUN, TEST_RUNNING, TEST_PASS, TEST_FAIL };

struct ChecklistItem {
  const char* name;
  TestResult result;
  String note;
  uint32_t lastMs;
};

ChecklistItem checklist[TEST_COUNT] = {
  {"EGT", TEST_NOT_RUN, "Not run", 0},
  {"RPM_NOISE", TEST_NOT_RUN, "Not run", 0},
  {"IGN_PULSE", TEST_NOT_RUN, "Not run", 0},
  {"STARTER", TEST_NOT_RUN, "Not run", 0},
  {"STARTER_IGN_EMI", TEST_NOT_RUN, "Not run", 0},
  {"VALVE1_START", TEST_NOT_RUN, "Not run", 0},
  {"VALVE2_MAIN", TEST_NOT_RUN, "Not run", 0},
  {"PUMP_PRIME", TEST_NOT_RUN, "Not run", 0},
  {"KILL_SWITCH", TEST_NOT_RUN, "Not confirmed", 0}
};

TestId activeTest = TEST_NONE;
uint32_t activeTestEndMs = 0;
uint32_t manualPumpOffAtMs = 0;


// USER button debounce/state
bool btnRawLastPressed = false;
bool btnStablePressed = false;
uint32_t btnLastRawChangeMs = 0;
uint32_t btnPressedAtMs = 0;
bool btnActionDone = false;

void writeActiveDigital(uint8_t pin, bool on, bool activeHigh) { digitalWrite(pin, (on == activeHigh) ? HIGH : LOW); }
const char* modeName(EcuMode m) { switch(m){case MODE_WAITING:return "WAITING";case MODE_STARTING:return "STARTING";case MODE_IDLING:return "IDLING";case MODE_OPERATING:return "OPERATING";case MODE_COOLDOWN:return "COOLDOWN";case MODE_ABORTED:return "ABORTED";default:return "UNKNOWN";} }
const char* stageName(StartStage s) { switch(s){case ST_NONE:return "NONE";case ST_PURGE:return "PURGE";case ST_SPINUP_PREHEAT:return "SPINUP_PREHEAT";case ST_INTRO_FUEL:return "INTRO_FUEL";case ST_LIGHTOFF:return "LIGHTOFF";case ST_ACCEL_TO_IDLE:return "ACCEL_TO_IDLE";default:return "UNKNOWN";} }
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

const char* rpmEdgeName() {
  return (rpmEdgeMode == FALLING) ? "FALLING" : "RISING";
}

bool rpmAtRestGuardCondition() {
  // Confirmed on bench: with outputs OFF and the engine truly stationary, GPIO33
  // can still show a steady-looking ~570-620 RPM from ambient EMI/pickup - low
  // jitter, so classifyRpmNoise() alone accepts it as CLEAN. The rest guard is the
  // only thing that catches this (real physical rest cannot be told apart from
  // noise by signal shape alone), so it must stay ON by default. rpmCalMode is an
  // explicit, operator-armed opt-out for bench calibration (spinning the sensor by
  // hand while WAITING) - see "set rpmcal on/off".
  if (rpmCalMode) return false;
  return ecuMode == MODE_WAITING && startStage == ST_NONE &&
         startUs <= (int)RPM_REST_GUARD_MAX_US &&
         pumpUs <= (int)RPM_REST_GUARD_MAX_US &&
         !ignCmd && !valve1Cmd && !valve2Cmd;
}

bool rpmNoiseBlocksStart() {
  return rpmData.noise == RPM_REST_NOISE || rpmData.noise == RPM_NOISY;
}

bool rpmMeasurementUsable() {
  return rpmData.signalRecent && rpmData.rpm > 0.0f &&
         rpmData.noise != RPM_NOISY && rpmData.noise != RPM_REST_NOISE;
}

const char* rpmSignalName() {
  if (rpmData.noise == RPM_REST_NOISE) return "REST_NOISE";
  if (rpmData.noise == RPM_NOISY) return "NOISY";
  return rpmData.signalRecent ? "OK" : "LOST";
}

RpmNoiseLevel classifyRpmNoise(bool recent, uint32_t raw, uint32_t accepted, uint32_t rejected,
                               float rejectPct, float jitterPct, float rpmDiffPct, uint32_t intervals) {
  if (!recent && raw == 0) return RPM_NO_SIGNAL;
  // raw>1 (not just the single priming edge after a reset) with nothing accepted =
  // pulses arriving but all rejected as glitches -> genuinely noisy. A lone first
  // edge (raw==1, accepted==0) is the normal post-reset priming case, not noise.
  if (raw > 1 && accepted == 0) return RPM_NOISY;
  if (rejected >= 3 || rejectPct > 20.0f ||
      (intervals >= 5 && jitterPct > 30.0f) ||
      (rpmDiffPct > 30.0f)) return RPM_NOISY;
  if (rejected > 0 || rejectPct > 5.0f ||
      (intervals >= 3 && jitterPct > 15.0f) ||
      (rpmDiffPct > 15.0f)) return RPM_WARN;
  return RPM_CLEAN;
}

void attachRpmInterrupt() {
  detachInterrupt(PIN_RPM);
  attachInterrupt(digitalPinToInterrupt(PIN_RPM), rpmISR, rpmEdgeMode);
}


String egtFaultString(uint8_t f) {
  if (f == 0) return "SPI/WIRING";
  String s = "";
  if (f & MAX31855_FAULT_OPEN) s += "OPEN ";
  if (f & MAX31855_FAULT_SHORT_GND) s += "SHORT_GND ";
  if (f & MAX31855_FAULT_SHORT_VCC) s += "SHORT_VCC ";
  s.trim(); return s;
}

float flowFromUs(int us) {
  if (us <= kPumpMap[0].us) return kPumpMap[0].mlMin;
  if (us >= kPumpMap[kPumpMapCount - 1].us) return kPumpMap[kPumpMapCount - 1].mlMin;
  for (size_t i = 1; i < kPumpMapCount; i++) if (us <= kPumpMap[i].us) {
    float x0 = kPumpMap[i-1].us, x1 = kPumpMap[i].us, y0 = kPumpMap[i-1].mlMin, y1 = kPumpMap[i].mlMin;
    return y0 + ((float)us - x0) * (y1 - y0) / (x1 - x0);
  }
  return kPumpMap[kPumpMapCount - 1].mlMin;
}

int usFromFlow(float mlMin) {
  if (mlMin <= kPumpMap[0].mlMin) return kPumpMap[0].us;
  if (mlMin >= kPumpMap[kPumpMapCount - 1].mlMin) return kPumpMap[kPumpMapCount - 1].us;
  for (size_t i = 1; i < kPumpMapCount; i++) if (mlMin <= kPumpMap[i].mlMin) {
    float y0 = kPumpMap[i-1].mlMin, y1 = kPumpMap[i].mlMin, x0 = kPumpMap[i-1].us, x1 = kPumpMap[i].us;
    return (int)lroundf(x0 + (mlMin - y0) * (x1 - x0) / (y1 - y0));
  }
  return kPumpMap[kPumpMapCount - 1].us;
}

void applyOutputs() {
  pumpUs = constrain(pumpUs, ESC_MIN_US, ESC_MAX_US);
  startUs = constrain(startUs, ESC_MIN_US, ESC_MAX_US);
  // Chi goi ledcWrite() khi gia tri thuc su doi. applyOutputs() duoc goi moi
  // vong loop(), va ledcWrite() lap lai khong can thiet (gia tri khong doi)
  // co the chiem CPU/critical-section toi muc anh huong ISR dem xung RPM -
  // da xac nhan bang TEST_RPM_RAWCOUNT.ino (trần cứng RAW_EDGES/s bien mat
  // sau khi bo goi lai khong can thiet nay). escAttach() chi goi 1 lan trong
  // setup() nen cache nay khong bi lech so voi trang thai LEDC thuc te.
  static int lastPumpUs = -1;
  static int lastStartUs = -1;
  if (pumpUs != lastPumpUs) { escWriteUs(PIN_ESC_PUMP, LEDC_CH_PUMP, pumpUs, escPumpPeriodUs); lastPumpUs = pumpUs; }
  if (startUs != lastStartUs) { escWriteUs(PIN_ESC_START, LEDC_CH_START, startUs, escStartPeriodUs); lastStartUs = startUs; }
  writeActiveDigital(PIN_IGN, ignCmd, IGN_ACTIVE_HIGH);
  writeActiveDigital(PIN_VALVE_1, valve1Cmd, VALVE_ACTIVE_HIGH);
  writeActiveDigital(PIN_VALVE_2, valve2Cmd, VALVE_ACTIVE_HIGH);
}

void fuelValvesAuto(bool on) {
  // VALVE1 (Start solenoid valve): only fed by this function during the
  // ignition/accel-to-idle sequence, force-closed the instant normal running
  // (IDLING/OPERATING) is reached so the main circuit alone carries fuel from
  // then on. In WAITING/ABORTED (bench/manual tests) it is left untouched
  // here - it may already be open via a separate manual "valve1 on" command.
  if (ecuMode == MODE_STARTING) valve1Cmd = on;
  else if (ecuMode == MODE_IDLING || ecuMode == MODE_OPERATING) valve1Cmd = false;
  // VALVE2 (Main oil valve): open whenever fuel is commanded, matching the
  // EnJet manual's "Oil Pump Test" behavior - but only if valve1 isn't
  // already providing a fuel path (one open valve is enough; don't open both).
  valve2Cmd = on && !valve1Cmd;
}
void forceSafeOutputs() { fuelTargetRpm = 0; fuelTargetUs = ESC_SAFE_US; pumpUs = ESC_SAFE_US; startUs = ESC_SAFE_US; ignCmd = valve1Cmd = valve2Cmd = false; applyOutputs(); }
void enterMode(EcuMode m) {
  // Anchor the flameout grace to the FIRST stable idle of this run so it is not
  // re-armed by later IDLING<->OPERATING throttle toggles (which would mask a
  // real flameout occurring during throttle changes).
  if (ecuMode == MODE_STARTING && m == MODE_IDLING) runningSinceMs = millis();
  if (ecuMode != m) addLog(String("MODE -> ") + modeName(m));
  ecuMode = m;
  modeEnteredMs = millis();
}

void enterStage(StartStage s) {
  if (startStage != s) addLog(String("STAGE -> ") + stageName(s));
  startStage = s;
  stageEnteredMs = millis();
}
void enterWaitingSafe() { dryStartActive = false; cooldownAfterAbort = false; enterMode(MODE_WAITING); enterStage(ST_NONE); throttlePct = 0; forceSafeOutputs(); }

void enterCooldown(bool afterAbort, const String& reason) {
  dryStartActive = false;
  cooldownAfterAbort = afterAbort;
  fuelTargetRpm = 0;
  fuelTargetUs = ESC_SAFE_US;
  pumpUs = ESC_SAFE_US;
  ignCmd = false;
  fuelValvesAuto(false);
  startUs = constrain(cfg.cooldownStarterUs, ESC_SAFE_US, ESC_MAX_US);
  applyOutputs();
  enterMode(MODE_COOLDOWN);
  enterStage(ST_NONE);
  addLog((afterAbort ? String("ABORT -> COOLDOWN: ") : String("SOFT STOP -> COOLDOWN: ")) + reason);
}

void abortAll(const String& reason) {
  lastAbortReason = reason;
  abortAcknowledged = false;
  // Snapshot RPM/EGT/fuel/outputs at the instant of the fault, before enterCooldown()
  // zeroes fuelTargetUs/pumpUs/ignCmd/startUs. Without this, the logged event line
  // shows the already-safed outputs instead of the values that caused the abort.
  addLog(String("ABORT_SNAPSHOT reason=") + reason);
  Serial.println();
  Serial.print("ABORT: ");
  Serial.print(reason);
  Serial.println(" -> COOLDOWN (type clearabort to acknowledge before re-arm)");
  Serial.println();
  enterCooldown(true, reason);
}

void requestStop() {
  // SOFT STOP only applies while the engine is actually running. Without this
  // guard, `stop` from MODE_ABORTED would wipe lastAbortReason, re-spin the
  // starter, and leak ABORTED -> WAITING, bypassing the clearabort interlock.
  if (ecuMode != MODE_STARTING && ecuMode != MODE_IDLING && ecuMode != MODE_OPERATING) {
    Serial.println("STOP ignored: only while STARTING/IDLING/OPERATING (use clearabort to leave ABORTED).");
    return;
  }
  lastAbortReason = "NONE";
  enterCooldown(false, "SOFT_STOP");
  Serial.println("STOP requested -> COOLDOWN");
}

bool isStage2Armed() {
  if (!stage2Armed) return false;
  if (millis() > stage2ArmUntilMs) { stage2Armed = false; Serial.println("STAGE2 AUTO-DISARMED."); return false; }
  return true;
}
void armStage2() {
  if (rpmCalMode) { Serial.println("ERROR: cannot ARM while RPM_CAL_MODE is ON (rest-guard disabled). Type 'set rpmcal off' first."); return; }
  stage2Armed = true; stage2ArmUntilMs = millis() + STAGE2_ARM_TIME_MS; addLog("ARMED 10s"); Serial.println("WARNING: STAGE2 ARMED FOR 10 SECONDS");
}
void stage2Off() {
  if (ecuMode == MODE_ABORTED) {
    // Outputs are already safe in ABORTED. SAFE OFF must NOT be a backdoor that
    // clears the fault and drops the "acknowledge before re-arm" interlock;
    // require an explicit clearabort (which sets abortAcknowledged).
    forceSafeOutputs();
    Serial.println("SAFE OFF: already safe in ABORTED. Use 'clearabort' to acknowledge the fault before re-arm.");
    return;
  }
  if (ecuMode == MODE_COOLDOWN && cooldownAfterAbort) {
    // This cooldown is heading to ABORTED. Do not let SAFE OFF short-circuit it to
    // WAITING (that would wipe the fault reason, skip the clearabort interlock, AND
    // cut the cooling airflow on a hot engine). Let the cooldown finish -> ABORTED.
    Serial.println("SAFE OFF ignored during post-abort cooldown. It will finish -> ABORTED; use 'clearabort' to acknowledge.");
    return;
  }
  stage2Armed = false;
  manualIgnOffAtMs = manualStartOffAtMs = manualPumpOffAtMs = 0;
  manAutoActive = false;
  activeTest = TEST_NONE; enterWaitingSafe(); addLog("SAFE OFF"); Serial.println("STAGE2 OFF: outputs safe.");
}


bool egtSafeForAbortClear() {
  // Automatic / remote clear: allowed only when the thermocouple is valid AND cool.
  return egt.ok && egt.c <= (cfg.ignitionThresholdC - 10);
}

bool egtAllowsDeliberateAbortClear() {
  // Deliberate operator clear (physical button hold, or "clearabort force"):
  // if the thermocouple is readable it must be cool, but if the sensor is dead we
  // cannot read temperature at all. Blocking on egt.ok would trap the ECU in
  // ABORTED forever when the thermocouple breaks, so we trust the physically
  // present operator here. A subsequent start is still forced into dry-start mode
  // (starter only, fuel/ignition OFF) while egt.ok is false, so this cannot open
  // fuel into a hot engine.
  if (!egt.ok) return true;
  return egt.c <= (cfg.ignitionThresholdC - 10);
}

bool fuelCommandBlockedByHotEgt() {
  // Refuse to open fuel into a demonstrably hot engine, e.g. a bench pump test issued
  // right after an OVER_TEMP abort while the engine is still above cooldown target.
  // If the sensor is unreadable we cannot verify temperature; the pump test is a
  // bench-only procedure with its own operator warning, so it is allowed in that case.
  return egt.ok && egt.c > cfg.cooldownTargetC;
}

void startIdleFromButton() {
  String why;

  // Button start is a deliberate 2-step action: hold 2s to ARM, then hold 3s to START.
  // It should not require the Serial-only "autostart on" flag to be left enabled.
  bool oldAuto = cfg.autoStartEnabled;
  cfg.autoStartEnabled = true;
  bool ok = canStartAutoIdle(why);
  cfg.autoStartEnabled = oldAuto;

  if (!ok) {
    Serial.print("USER_BTN START BLOCKED: ");
    Serial.println(why);
    return;
  }

  stage2Armed = false;
  Serial.println("USER_BTN: START IDLE requested.");
  beginAutoIdle();
  // Physical-button run: there is no Serial/Web link to prove alive, so the comm
  // watchdog does not apply. It re-engages the moment any Serial/Web command arrives.
  runStartedByButton = true;
}

void userButtonPressed() {
  if (ecuMode == MODE_STARTING || ecuMode == MODE_IDLING || ecuMode == MODE_OPERATING) {
    btnActionDone = true;
    Serial.println("USER_BTN: SOFT STOP.");
    requestStop();
  }
}

void userButtonReleased(uint32_t heldMs) {
  if (btnActionDone) return;

  if (ecuMode == MODE_WAITING) {
    Serial.print("USER_BTN SHORT: status/log marker, held=");
    Serial.print(heldMs);
    Serial.println(" ms");
    printStatus(true);
    return;
  }

  if (ecuMode == MODE_ABORTED) {
    Serial.print("USER_BTN SHORT while ABORTED, held=");
    Serial.print(heldMs);
    Serial.println(" ms. Hold 2s to clear if EGT is safe.");
    printStatus(true);
    return;
  }
}

void handleUserButton() {
  uint32_t now = millis();
  bool rawPressed = USER_BTN_ACTIVE_LOW ? (digitalRead(PIN_USER_BTN) == LOW) : (digitalRead(PIN_USER_BTN) == HIGH);

  if (rawPressed != btnRawLastPressed) {
    btnRawLastPressed = rawPressed;
    btnLastRawChangeMs = now;
  }

  if ((now - btnLastRawChangeMs) >= BTN_DEBOUNCE_MS && rawPressed != btnStablePressed) {
    btnStablePressed = rawPressed;

    if (btnStablePressed) {
      btnPressedAtMs = now;
      btnActionDone = false;
      userButtonPressed();
    } else {
      uint32_t heldMs = (btnPressedAtMs == 0) ? 0 : (now - btnPressedAtMs);
      userButtonReleased(heldMs);
      btnPressedAtMs = 0;
      btnActionDone = false;
    }
  }

  if (!btnStablePressed || btnActionDone) return;

  uint32_t heldMs = now - btnPressedAtMs;

  if (ecuMode == MODE_WAITING) {
    if (stage2Armed) {
      if (heldMs >= BTN_START_HOLD_MS) {
        btnActionDone = true;
        startIdleFromButton();
      }
    } else {
      if (heldMs >= BTN_ARM_HOLD_MS) {
        btnActionDone = true;
        Serial.println("USER_BTN: ARM requested.");
        armStage2();
      }
    }
    return;
  }

  if (ecuMode == MODE_ABORTED) {
    if (heldMs >= BTN_CLEAR_ABORT_HOLD_MS) {
      btnActionDone = true;
      if (egtAllowsDeliberateAbortClear()) {
        lastAbortReason = "CLEARED_BY_BUTTON";
        abortAcknowledged = true;
        if (!egt.ok) Serial.println("USER_BTN: WARNING EGT sensor faulty - temperature not verified. Next start will be DRY only.");
        enterWaitingSafe();
        Serial.println("USER_BTN: ABORT cleared, ECU -> WAITING.");
      } else {
        Serial.print("USER_BTN CLEAR BLOCKED: EGT still hot. EGT=");
        Serial.print(egt.c, 1);
        Serial.println("C");
      }
    }
  }
}


void writeStatusLed(bool on) {
  if (STATUS_LED_ACTIVE_LOW) digitalWrite(PIN_STATUS_LED, on ? LOW : HIGH);
  else digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}

void blinkStatusLed(uint32_t intervalMs) {
  static uint32_t lastToggleMs = 0;
  static bool ledOn = false;

  uint32_t now = millis();
  if (now - lastToggleMs >= intervalMs) {
    lastToggleMs = now;
    ledOn = !ledOn;
    writeStatusLed(ledOn);
  }
}

void updateStatusLed() {
  // NodeMCU-32S onboard LED is active-low on the tested board:
  // LOW = ON, HIGH = OFF.
  // Do not connect any external load to GPIO2; it is a boot strap pin.

  // Component test running: very fast blink.
  if (activeTest != TEST_NONE) {
    blinkStatusLed(80);
    return;
  }

  // Abort/fault: fast blink.
  if (ecuMode == MODE_ABORTED) {
    blinkStatusLed(100);
    return;
  }

  // Armed but not starting yet: quick blink.
  if (stage2Armed && ecuMode == MODE_WAITING) {
    blinkStatusLed(200);
    return;
  }

  // Starting or operating: solid ON.
  if (ecuMode == MODE_STARTING || ecuMode == MODE_OPERATING) {
    writeStatusLed(true);
    return;
  }

  // Stable idle: medium blink.
  if (ecuMode == MODE_IDLING) {
    blinkStatusLed(500);
    return;
  }

  // Cooldown: faster-than-waiting blink.
  if (ecuMode == MODE_COOLDOWN) {
    blinkStatusLed(300);
    return;
  }

  // Waiting/safe: slow heartbeat blink.
  if (ecuMode == MODE_WAITING) {
    blinkStatusLed(1000);
    return;
  }

  writeStatusLed(false);
}

void stepPumpToward(int targetUs, uint32_t upDelayMs, uint32_t downDelayMs) {
  targetUs = constrain(targetUs, ESC_SAFE_US, cfg.maxFuelUs);
  uint32_t now = millis();
  if (targetUs > pumpUs && now - lastPumpStepMs >= upDelayMs) { pumpUs++; lastPumpStepMs = now; }
  else if (targetUs < pumpUs && now - lastPumpStepMs >= downDelayMs) { pumpUs--; lastPumpStepMs = now; }
}

void setFuelTargetUs(int us) {
  fuelTargetUs = constrain(us, ESC_SAFE_US, cfg.maxFuelUs);
}

bool egtAllowsFuelIncrease() {
  if (!egt.ok) return false;
  if (egt.c >= cfg.maxEgtC) return false;
  // During the start sequence a fast EGT rise at light-off is normal and the engine
  // NEEDS fuel to accelerate to idle; only the absolute maxEgtC limits here (the
  // OVER_TEMP abort with its short look-ahead is the safety net). Gradient + 3s
  // projected-temp limiting apply only in steady IDLING/OPERATING.
  if (ecuMode == MODE_STARTING) return true;
  if (egt.gradientCps >= cfg.maxTempGradientCps) return false;
  if ((egt.c + 3.0f * egt.gradientCps) >= cfg.maxEgtC) return false; // 3s look-ahead like Ardu_ECU
  return true;
}

bool egtRequestsFuelCut() {
  if (!egt.ok) return true;
  if (egt.c >= cfg.maxEgtC) return true;
  if (ecuMode == MODE_STARTING) return false;  // see egtAllowsFuelIncrease(): start rise is expected
  if (egt.gradientCps >= cfg.maxTempGradientCps) return true;
  if ((egt.c + 3.0f * egt.gradientCps) >= cfg.maxEgtC) return true;
  return false;
}

void updateFuelClosedLoopToRpm(int targetRpm, int minUs, int maxUs, bool lowRpmRegion) {
  minUs = constrain(minUs, ESC_SAFE_US, cfg.maxFuelUs);
  maxUs = constrain(maxUs, minUs, cfg.maxFuelUs);
  fuelTargetRpm = targetRpm;
  fuelTargetUs = constrain(fuelTargetUs, minUs, maxUs);

  bool rpmOk = rpmMeasurementUsable() && rpmData.rpm > 1000.0f;
  bool rpmTooHigh = rpmOk && ((rpmData.rpm > (float)(targetRpm + cfg.rpmTolerance)) || (rpmData.rpm >= (float)cfg.maxRpm));
  bool rpmTooLow = rpmOk && (rpmData.rpm < (float)targetRpm);
  bool egtCut = egtRequestsFuelCut();

  if (egtCut || rpmTooHigh) {
    fuelTargetUs -= cfg.fuelCutStepUs;
  } else if (rpmTooLow && egtAllowsFuelIncrease()) {
    fuelTargetUs += cfg.fuelStepUs;
  } else {
    // Ardu_ECU-style: when target is reached, do not keep a queued fuel ramp.
    fuelTargetUs = pumpUs;
  }

  fuelTargetUs = constrain(fuelTargetUs, minUs, maxUs);

  if (egtCut && pumpUs > minUs) {
    // Over-temp / over-gradient is an emergency. The normal ramp only moves pumpUs
    // 1us per decel-delay (~2us/s in the low region), far too slow to actually
    // reduce fuel before EGT hits the hard OVER_TEMP abort. Here step the ACTUAL
    // output down by fuelCutStepUs at the fast decel rate so soft protection can
    // keep up (~40us/s), while still bounded by the hard abort.
    uint32_t now = millis();
    if (now - lastPumpStepMs >= cfg.decelStepDelayMs) {
      pumpUs = (int)max(minUs, pumpUs - cfg.fuelCutStepUs);
      lastPumpStepMs = now;
    }
  } else {
    stepPumpToward(fuelTargetUs,
                   lowRpmRegion ? cfg.lowAccelStepDelayMs : cfg.accelStepDelayMs,
                   lowRpmRegion ? cfg.lowDecelStepDelayMs : cfg.decelStepDelayMs);
  }
}

bool rpmSignalRecentWithin(uint32_t timeoutMs) {
  uint32_t lastUs;
  noInterrupts();
  lastUs = isrLastAcceptedPulseUs;
  interrupts();
  return lastUs != 0 && ((uint32_t)(micros() - lastUs) <= timeoutMs * 1000UL);
}

void IRAM_ATTR rpmISR() {
  uint32_t nowUs = micros();
  if (nowUs == 0) nowUs = 1;   // 0 is the "no edge yet" sentinel; avoid it at the ~71min micros() wrap
  isrRawEdges++;

  if (isrLastRawEdgeUs == 0) {
    // First edge after reset: record timestamp only; we need at least one interval before accepting.
    isrLastRawEdgeUs = nowUs;
    return;
  }

  uint32_t dtRawUs = nowUs - isrLastRawEdgeUs;
  isrLastRawEdgeUs = nowUs;

  // (1) Fixed min-pulse (quiet-period) de-glitch: an edge must be preceded by at
  //     least filterUs of silence since the previous RAW edge. This fully rejects
  //     dense EMI bursts (edges spaced closer than filterUs).
  uint32_t filterUs = rpmMinPulseUs;
  if (dtRawUs < filterUs) {
    isrRejectedEdges++;
    return;
  }

  if (isrLastAcceptedPulseUs != 0) {
    uint32_t dtAcceptedUs = nowUs - isrLastAcceptedPulseUs;
    // (2) Adaptive mask: once a valid period is known, also reject any edge that
    //     arrives within half of the last accepted period. A turbine cannot double
    //     its speed within a single revolution, so genuine acceleration pulses are
    //     preserved, while an isolated EMI spike that lands after a quiet gap (and
    //     would otherwise pass the fixed filter) is rejected instead of being
    //     counted as a phantom high-RPM pulse.
    uint32_t maskUs = filterUs;
    if (isrLastPeriodUs > 0 && (isrLastPeriodUs >> 1) > maskUs) maskUs = (isrLastPeriodUs >> 1);
    if (dtAcceptedUs < maskUs) {
      isrRejectedEdges++;
      return;
    }
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
  isrLastRawEdgeUs = 0;
  isrLastAcceptedPulseUs = 0;
  isrLastPeriodUs = 0;
  isrRawEdges = 0;
  isrAcceptedPulses = 0;
  isrRejectedEdges = 0;
  isrAcceptedIntervals = 0;
  isrSumDtUs = 0;
  isrSumDtSqUs = 0;
  isrMinDtUs = 0xFFFFFFFFUL;
  isrMaxDtUs = 0;
  interrupts();
  rpmData = RpmState();
  // Grace so a stats reset (isrLastAcceptedPulseUs zeroed until the next edge) can't
  // momentarily read as "signal lost" and false-abort a running engine.
  rpmStatsResetAtMs = millis();
  Serial.println("RPM stats reset.");
}

void updateRpm() {
  uint32_t nowMs = millis();
  if (nowMs - rpmData.lastComputedMs < RPM_SAMPLE_MS) return;

  uint32_t accepted, raw, rejected, validN, minDt, maxDt, lastPulseUs, lastPeriodUs, filterUs;
  uint64_t sumDt, sumDtSq;

  noInterrupts();
  accepted = isrAcceptedPulses;
  raw = isrRawEdges;
  rejected = isrRejectedEdges;
  validN = isrAcceptedIntervals;
  sumDt = isrSumDtUs;
  sumDtSq = isrSumDtSqUs;
  minDt = isrMinDtUs;
  maxDt = isrMaxDtUs;
  lastPulseUs = isrLastAcceptedPulseUs;
  lastPeriodUs = isrLastPeriodUs;
  filterUs = rpmMinPulseUs;

  isrAcceptedPulses = 0;
  isrRawEdges = 0;
  isrRejectedEdges = 0;
  isrAcceptedIntervals = 0;
  isrSumDtUs = 0;
  isrSumDtSqUs = 0;
  isrMinDtUs = 0xFFFFFFFFUL;
  isrMaxDtUs = 0;
  interrupts();

  // Sample the clock AFTER the critical section. lastPulseUs was written by an ISR
  // that has already run, so a nowUs taken here is always >= lastPulseUs. This
  // prevents a uint32_t wraparound in the signalRecent computation below when the
  // ISR fires between capturing the time and entering the critical section.
  uint32_t nowUs = micros();
  if (nowUs == 0) nowUs = 1;   // 0 is the "first window" sentinel for lastComputedUs; avoid it at the micros() wrap
  uint32_t windowUs = (rpmData.lastComputedUs == 0) ? (RPM_SAMPLE_MS * 1000UL) : (nowUs - rpmData.lastComputedUs);
  rpmData.lastComputedUs = nowUs;
  rpmData.lastComputedMs = nowMs;

  rpmData.acceptedWindow = accepted;
  rpmData.rawEdges = raw;
  rpmData.rejectedEdges = rejected;
  rpmData.validIntervals = validN;
  rpmData.lastAcceptedPulseUsSnapshot = lastPulseUs;
  rpmData.lastPeriodUs = lastPeriodUs;
  rpmData.minIntervalUs = (minDt == 0xFFFFFFFFUL) ? 0 : minDt;
  rpmData.maxIntervalUs = maxDt;
  rpmData.filterUs = filterUs;
  rpmData.pinLevel = digitalRead(PIN_RPM);

  rpmData.restGuardActive = rpmAtRestGuardCondition();

  rpmData.signalRecent = lastPulseUs != 0 && ((uint32_t)(nowUs - lastPulseUs) <= RPM_SIGNAL_TIMEOUT_MS * 1000UL);
  rpmData.restPulseNoise = rpmData.restGuardActive && (raw > 0 || accepted > 0 || rpmData.signalRecent);

  rpmData.rejectPct = (raw > 0) ? ((float)rejected * 100.0f / (float)raw) : 0.0f;
  rpmData.rpmWindow = (accepted > 0 && windowUs > 0) ?
                      ((float)accepted * 60000000.0f / ((float)windowUs * (float)cfg.pulsesPerRev)) : 0.0f;

  rpmData.rpmPeriod = 0.0f;
  if (rpmData.signalRecent && lastPeriodUs > 0) {
    rpmData.rpmPeriod = 60000000.0f / ((float)lastPeriodUs * (float)cfg.pulsesPerRev);
  }

  if (validN > 0) {
    rpmData.avgIntervalUs = (float)sumDt / (float)validN;
    // Jitter = coefficient of variation (stddev/mean), not (max-min)/mean. A single
    // stray long interval in the window skews max-min heavily regardless of how
    // many clean samples surround it; stddev divides the outlier's contribution by
    // validN, so one glitch no longer swamps an otherwise-clean signal into a false
    // RPM_WARN/RPM_NOISY.
    double meanUs = (double)sumDt / (double)validN;
    double meanSqUs = (double)sumDtSq / (double)validN;
    double variance = meanSqUs - meanUs * meanUs;
    if (variance < 0.0) variance = 0.0;  // fp rounding guard
    float stddevUs = sqrtf((float)variance);
    rpmData.jitterPct = (rpmData.avgIntervalUs > 0.0f) ? (stddevUs * 100.0f / rpmData.avgIntervalUs) : 0.0f;
  } else if (!rpmData.signalRecent) {
    rpmData.avgIntervalUs = 0.0f;
    rpmData.jitterPct = 0.0f;
  }

  rpmData.rpmDiffPct = 0.0f;
  if (rpmData.rpmWindow > 0.0f && rpmData.rpmPeriod > 0.0f) {
    float base = max(rpmData.rpmWindow, rpmData.rpmPeriod);
    rpmData.rpmDiffPct = fabsf(rpmData.rpmWindow - rpmData.rpmPeriod) * 100.0f / base;
  }

  if (rpmData.restPulseNoise) {
    // Engine commanded OFF and rpmCalMode not armed. Confirmed on bench: ambient
    // EMI/pickup on GPIO33 can look like a steady, low-jitter ~570-620 RPM signal
    // even with the sensor stationary, so classifyRpmNoise() alone would accept it
    // as CLEAN. Force RPM to zero here; raw/acc/per diagnostics are kept.
    rpmData.rpm = 0.0f;
    rpmData.signalRecent = false;
    rpmData.noise = RPM_REST_NOISE;
  } else {
    // Outside the rest-guard (running, or rpmCalMode armed for bench calibration):
    // period RPM reacts quickly; window RPM remains as a diagnostic cross-check.
    if (rpmData.signalRecent && rpmData.rpmPeriod > 0.0f) rpmData.rpm = rpmData.rpmPeriod;
    else if (accepted > 0) rpmData.rpm = rpmData.rpmWindow;
    else rpmData.rpm = 0.0f;

    rpmData.noise = classifyRpmNoise(rpmData.signalRecent, raw, accepted, rejected,
                                     rpmData.rejectPct, rpmData.jitterPct,
                                     rpmData.rpmDiffPct, validN);
  }
}

void updateEgt() {
  uint32_t nowMs = millis(); if (nowMs - egt.lastReadMs < EGT_READ_PERIOD_MS) return;
  egt.lastReadMs = nowMs;
  // Read fault bits first; readCelsius() then re-reads the chip once more (two SPI transactions
  // is unavoidable with this library). Using readError() first ensures the fault code comes from
  // the same chip state as the isnan() check that follows.
  uint8_t fault = thermo.readError();
  double tc = thermo.readCelsius();
  if (isnan(tc) || fault != 0) {
    egt.ok = false;
    egt.fault = fault;
    egt.gradientCps = 0;  // clear stale gradient so display shows 0 when sensor is invalid
    return;
  }
  if (egt.ok && !isnan(egt.c)) {
    float dtS = (float)(nowMs - egt.lastGoodMs) / 1000.0f;
    if (dtS > 0.05f) egt.gradientCps = ((float)tc - egt.c) / dtS;
  } else egt.gradientCps = 0;
  egt.prevC = egt.c; egt.c = (float)tc; egt.ok = true; egt.fault = 0; egt.lastGoodMs = nowMs;

  // Frozen-sensor heuristic (LOG ONLY - never aborts, to avoid false shutdowns on a
  // legitimately steady EGT). A live thermocouple always fluctuates during combustion;
  // if the reading is byte-for-byte unchanged for EGT_STUCK_WARN_MS while fuel is
  // flowing and the core is hot, warn once. A frozen-but-plausible value would not
  // trip the MAX31855 open/short fault path, so this is the only hint of it.
  static float egtStuckVal = NAN;
  static uint32_t egtStuckSinceMs = 0;
  static bool egtStuckWarned = false;
  bool combusting = (egt.c > (float)cfg.ignitionThresholdC) &&
                    ((pumpUs > ESC_SAFE_US + 10) || valve1Cmd || valve2Cmd);
  if (combusting && !isnan(egtStuckVal) && egt.c == egtStuckVal) {
    if (!egtStuckWarned && (nowMs - egtStuckSinceMs) >= EGT_STUCK_WARN_MS) {
      addLog("WARN: EGT frozen while combusting - check thermocouple");
      Serial.println("WARNING: EGT unchanged too long while fueled - possible frozen/stuck thermocouple.");
      egtStuckWarned = true;
    }
  } else {
    egtStuckVal = egt.c;
    egtStuckSinceMs = nowMs;
    egtStuckWarned = false;
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
  line.reserve(320);
  line += String(millis()); line += ",";
  line += type; line += ",";
  line += modeName(ecuMode); line += ",";
  line += stageName(startStage); line += ",";
  line += sdFloat(egt.ok ? egt.c : NAN, 1); line += ",";
  line += sdFloat(egt.ok ? egt.gradientCps : NAN, 1); line += ",";
  line += sdFloat(rpmData.rpm, 0); line += ",";
  line += String(fuelTargetRpm); line += ",";
  line += String(pumpUs); line += ",";
  line += sdFloat(flowFromUs(pumpUs), 1); line += ",";
  line += String(fuelTargetUs); line += ",";
  line += String(startUs); line += ",";
  line += (ignCmd ? "1" : "0"); line += ",";
  line += (valve1Cmd ? "1" : "0"); line += ",";
  line += (valve2Cmd ? "1" : "0"); line += ",";
  line += String(throttlePct); line += ",";
  line += sdCsvQuote(lastAbortReason); line += ",";
  line += sdCsvQuote(eventText);
  return line;
}

// Consecutive-failure counter for the SD backoff below. Reset on any good write.
static uint32_t sdConsecFail = 0;
static const uint32_t SD_MAX_CONSEC_FAIL = 5;

void sdAppendLine(const String& line) {
  if (!sdOk || !cfg.sdLoggingEnabled) return;

  File f = SD.open(sdLogPath, FILE_APPEND);
  if (!f) {
    sdWriteFailCount++;
    sdConsecFail++;
    if (sdWriteFailCount == 1 || sdWriteFailCount % 10 == 0) {
      Serial.print("SD LOG WRITE FAIL count="); Serial.println(sdWriteFailCount);
    }
    // Card pulled / failing: stop hammering SD.open() every telemetry cycle, which
    // would otherwise block the control loop for hundreds of ms twice a second.
    if (sdConsecFail >= SD_MAX_CONSEC_FAIL) {
      sdOk = false;
      Serial.println("SD DISABLED after repeated write failures (card removed/failing).");
      addLog("SD DISABLED (repeated write fail)");
    }
    return;
  }

  size_t n = f.println(line);
  f.close();
  if (n == 0) { sdWriteFailCount++; sdConsecFail++; }
  else sdConsecFail = 0;
}

void sdLogEvent(const String& msg) {
  if (!sdOk || !cfg.sdLoggingEnabled) return;
  // Snapshot the CSV line now (captures EGT/RPM/outputs at event time) but defer the
  // blocking write. flushSdEventQueue() runs later in loop(), after the safety checks.
  uint8_t next = (sdEventQHead + 1) % SD_EVENT_QUEUE;
  if (next == sdEventQTail) {
    // Queue full: drop the oldest to keep the most recent events.
    sdEventQTail = (sdEventQTail + 1) % SD_EVENT_QUEUE;
    sdEventDropCount++;
  }
  sdEventQueue[sdEventQHead] = sdCsvLine("EVENT", msg);
  sdEventQHead = next;
}

void flushSdEventQueue() {
  while (sdEventQTail != sdEventQHead) {
    sdAppendLine(sdEventQueue[sdEventQTail]);
    sdEventQueue[sdEventQTail] = "";  // release the String buffer
    sdEventQTail = (sdEventQTail + 1) % SD_EVENT_QUEUE;
  }
}

void updateSdTelemetry() {
  if (!sdOk || !cfg.sdLoggingEnabled) return;

  bool activeForLog = (ecuMode == MODE_STARTING || ecuMode == MODE_IDLING ||
                       ecuMode == MODE_OPERATING || ecuMode == MODE_COOLDOWN ||
                       activeTest != TEST_NONE);
  if (!activeForLog) return;

  uint32_t now = millis();
  if (now - lastSdTelemetryMs < SD_LOG_PERIOD_MS) return;
  lastSdTelemetryMs = now;

  sdAppendLine(sdCsvLine("DATA", ""));
}

void printSdStatus() {
  Serial.println("===== SD LOG STATUS =====");
  Serial.print("sdOk="); Serial.println(sdOk ? "OK" : "FAIL/NOT_INIT");
  Serial.print("sdLoggingEnabled="); Serial.println(cfg.sdLoggingEnabled ? "ON" : "OFF");
  Serial.print("sdLogPath="); Serial.println(sdLogPath);
  Serial.print("sdSpiHz="); Serial.println(SD_SPI_HZ);
  Serial.print("sdWriteFailCount="); Serial.println(sdWriteFailCount);
  Serial.println("Pins: CS=13 SCK=14 MOSI=23 MISO=27");
  Serial.println("=========================");
}

// Mount the SD card once (idempotent). Used by BOTH config persistence and CSV
// logging, so config can load even when logging is disabled. Returns true if a
// usable card is present.
bool mountSd() {
  if (sdMounted) return sdOk;
  sdMounted = true;
  sdSPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (!SD.begin(PIN_SD_CS, sdSPI, SD_SPI_HZ)) {
    sdOk = false;
    Serial.println("SD: SD.begin() FAIL. Check FAT32 card and pins CS=13 SCK=14 MOSI=23 MISO=27.");
    return false;
  }
  if (SD.cardType() == CARD_NONE) {
    sdOk = false;
    Serial.println("SD: no card detected.");
    return false;
  }
  sdOk = true;
  return true;
}

static int clampCfgInt(long v, long lo, long hi) { return (int)(v < lo ? lo : (v > hi ? hi : v)); }

// Persist all tunable config to /ECUCFG.TXT as key=value lines, so the ECU can be
// configured once and reload on next power-up. Runtime/safety-sensitive state
// (autoStart, throttle, web) is intentionally NOT persisted (always boots safe).
bool saveConfigToSd() {
  if (!mountSd()) { Serial.println("SAVECFG: no SD card."); addLog("SAVECFG FAIL no SD"); return false; }
  SD.remove(CONFIG_PATH);   // clean overwrite (FILE_WRITE does not truncate on all cores)
  File f = SD.open(CONFIG_PATH, FILE_WRITE);
  if (!f) { Serial.println("SAVECFG: open FAIL."); addLog("SAVECFG FAIL open"); return false; }
  f.println("# ECU config - auto-loaded on boot. Edit via Web UI Settings tab.");
  f.print("idlerpm=");      f.println(cfg.idleRpm);
  f.print("maxrpm=");       f.println(cfg.maxRpm);
  f.print("rpmtol=");       f.println(cfg.rpmTolerance);
  f.print("maxegt=");       f.println(cfg.maxEgtC);
  f.print("maxgrad=");      f.println(cfg.maxTempGradientCps);
  f.print("ppr=");          f.println((int)cfg.pulsesPerRev);
  f.print("rpmfilter=");    f.println((uint32_t)rpmMinPulseUs);
  f.print("rpmedge=");      f.println(rpmEdgeName());
  f.print("introus=");      f.println(cfg.introFuelUs);
  f.print("idleus=");       f.println(cfg.idleFuelUs);
  f.print("maxus=");        f.println(cfg.maxFuelUs);
  f.print("pumptestus=");   f.println(cfg.pumpTestUs);
  f.print("purgeus=");      f.println(cfg.starterPurgeUs);
  f.print("spinus=");       f.println(cfg.starterSpinUs);
  f.print("assistus=");     f.println(cfg.starterAssistUs);
  f.print("accelms=");      f.println(cfg.accelStepDelayMs);
  f.print("decelms=");      f.println(cfg.decelStepDelayMs);
  f.print("lowaccelms=");   f.println(cfg.lowAccelStepDelayMs);
  f.print("lowdecelms=");   f.println(cfg.lowDecelStepDelayMs);
  f.print("drystartms=");   f.println(cfg.dryStartRunMs);
  f.print("acceltoidlems=");f.println(cfg.accelToIdleTimeoutMs);
  f.print("cooltarget=");   f.println(cfg.cooldownTargetC);
  f.print("coolstarter=");  f.println(cfg.cooldownStarterUs);
  f.print("coolminms=");    f.println(cfg.cooldownMinMs);
  f.print("cooltimeoutms=");f.println(cfg.cooldownTimeoutMs);
  f.print("commtimeout=");  f.println(cfg.commTimeoutMs);
  f.print("commwd=");       f.println(cfg.commWatchdogEnabled ? 1 : 0);
  f.print("checklist=");    f.println(cfg.requireChecklistForStart ? 1 : 0);
  f.print("egtdry=");       f.println(cfg.allowDryStartWhenEgtFault ? 1 : 0);
  f.print("sdlog=");        f.println(cfg.sdLoggingEnabled ? 1 : 0);
  f.print("rampfromus=");   f.println(cfg.startRampFromUs);
  f.print("ramptous=");     f.println(cfg.startRampToUs);
  f.print("rampstepms=");   f.println(cfg.startRampStepMs);
  f.print("escarmholdms="); f.println(cfg.escArmHoldMs);
  f.print("purgestablems=");f.println(cfg.purgeStableMs);
  f.print("ignarmrpm=");    f.println(cfg.ignArmRpm);
  f.print("spinuptimeoutms=");f.println(cfg.spinupRpmTimeoutMs);
  f.print("fueldelayms=");  f.println(cfg.fuelDelayMs);
  f.print("lightoffrise="); f.println(cfg.lightOffMinRiseCps);
  f.print("lightoffconfirmms=");f.println(cfg.lightOffConfirmMs);
  f.print("flameprovems="); f.println(cfg.flameProveMs);
  f.print("flameoutdropc=");f.println(cfg.flameOutDropC);
  f.print("accelholdms=");  f.println(cfg.accelHoldMs);
  f.print("accelstepus=");  f.println(cfg.accelStepUs);
  f.print("purgeoutms=");   f.println(cfg.purgeOutMs);
  f.print("startermaxrpm=");f.println(cfg.starterMaxRpm);
  f.print("hotspinminrpm=");f.println(cfg.hotSpinMinRpm);
  f.print("hotspinus=");    f.println(cfg.hotSpinUs);
  f.close();
  cfgFileOnSd = true;
  Serial.println("SAVECFG: OK -> /ECUCFG.TXT");
  addLog("CONFIG SAVED to SD");
  return true;
}

void applyConfigKV(const String& key, const String& val) {
  long n = val.toInt();
  if      (key == "idlerpm")      cfg.idleRpm = clampCfgInt(n, 10000, 60000);
  else if (key == "maxrpm")       cfg.maxRpm = clampCfgInt(n, 15000, 160000);
  else if (key == "rpmtol")       cfg.rpmTolerance = clampCfgInt(n, 500, 15000);
  else if (key == "maxegt")       cfg.maxEgtC = clampCfgInt(n, 400, 950);
  else if (key == "maxgrad")      cfg.maxTempGradientCps = clampCfgInt(n, 50, 1000);
  else if (key == "ppr")          cfg.pulsesPerRev = (n == 2) ? 2 : 1;
  else if (key == "rpmfilter")    rpmMinPulseUs = (uint32_t)clampCfgInt(n, 20, 5000);
  else if (key == "rpmedge")      rpmEdgeMode = (val == "FALLING") ? FALLING : RISING;
  else if (key == "introus")      cfg.introFuelUs = clampCfgInt(n, 1000, 1250);
  else if (key == "idleus")       cfg.idleFuelUs = clampCfgInt(n, 1000, 1270);
  else if (key == "maxus")        cfg.maxFuelUs = clampCfgInt(n, 1100, 1300);
  else if (key == "pumptestus")   cfg.pumpTestUs = clampCfgInt(n, 1000, 1225);
  else if (key == "purgeus")      cfg.starterPurgeUs = clampCfgInt(n, 1000, 1500);
  else if (key == "spinus")       cfg.starterSpinUs = clampCfgInt(n, 1000, 1500);
  else if (key == "assistus")     cfg.starterAssistUs = clampCfgInt(n, 1000, 1500);
  else if (key == "accelms")      cfg.accelStepDelayMs = clampCfgInt(n, 50, 2000);
  else if (key == "decelms")      cfg.decelStepDelayMs = clampCfgInt(n, 50, 2000);
  else if (key == "lowaccelms")   cfg.lowAccelStepDelayMs = clampCfgInt(n, 100, 3000);
  else if (key == "lowdecelms")   cfg.lowDecelStepDelayMs = clampCfgInt(n, 100, 3000);
  else if (key == "drystartms")   cfg.dryStartRunMs = (uint32_t)clampCfgInt(n, 1000, 15000);
  else if (key == "acceltoidlems")cfg.accelToIdleTimeoutMs = (uint32_t)clampCfgInt(n, 5000, 60000);
  else if (key == "cooltarget")   cfg.cooldownTargetC = clampCfgInt(n, 50, 250);
  else if (key == "coolstarter")  cfg.cooldownStarterUs = clampCfgInt(n, 1000, 1200);
  else if (key == "coolminms")    cfg.cooldownMinMs = (uint32_t)clampCfgInt(n, 1000, 30000);
  else if (key == "cooltimeoutms")cfg.cooldownTimeoutMs = (uint32_t)clampCfgInt(n, 5000, 120000);
  else if (key == "commtimeout")  cfg.commTimeoutMs = (uint32_t)clampCfgInt(n, 3000, 60000);
  else if (key == "commwd")       cfg.commWatchdogEnabled = (n != 0);
  else if (key == "checklist")    cfg.requireChecklistForStart = (n != 0);
  else if (key == "egtdry")       cfg.allowDryStartWhenEgtFault = (n != 0);
  else if (key == "sdlog")        cfg.sdLoggingEnabled = (n != 0);
  else if (key == "rampfromus")   cfg.startRampFromUs = clampCfgInt(n, 1000, 1400);
  else if (key == "ramptous")     cfg.startRampToUs = clampCfgInt(n, 1000, 1500);
  else if (key == "rampstepms")   cfg.startRampStepMs = (uint32_t)clampCfgInt(n, 10, 5000);
  else if (key == "escarmholdms") cfg.escArmHoldMs = (uint32_t)clampCfgInt(n, 0, 10000);
  else if (key == "purgestablems")cfg.purgeStableMs = (uint32_t)clampCfgInt(n, 0, 60000);
  else if (key == "ignarmrpm")    cfg.ignArmRpm = clampCfgInt(n, 500, 60000);
  else if (key == "spinuptimeoutms") cfg.spinupRpmTimeoutMs = (uint32_t)clampCfgInt(n, 5000, 180000);
  else if (key == "fueldelayms")  cfg.fuelDelayMs = (uint32_t)clampCfgInt(n, 0, 20000);
  else if (key == "lightoffrise") cfg.lightOffMinRiseCps = clampCfgInt(n, 0, 500);
  else if (key == "lightoffconfirmms") cfg.lightOffConfirmMs = (uint32_t)clampCfgInt(n, 0, 10000);
  else if (key == "flameprovems") cfg.flameProveMs = (uint32_t)clampCfgInt(n, 0, 20000);
  else if (key == "flameoutdropc") cfg.flameOutDropC = clampCfgInt(n, 1, 500);
  else if (key == "accelholdms")  cfg.accelHoldMs = (uint32_t)clampCfgInt(n, 200, 60000);
  else if (key == "accelstepus")  cfg.accelStepUs = clampCfgInt(n, 1, 50);
  else if (key == "purgeoutms")   cfg.purgeOutMs = (uint32_t)clampCfgInt(n, 0, 60000);
  else if (key == "startermaxrpm") cfg.starterMaxRpm = clampCfgInt(n, 3000, 60000);
  else if (key == "hotspinminrpm") cfg.hotSpinMinRpm = clampCfgInt(n, 0, 20000);
  else if (key == "hotspinus")    cfg.hotSpinUs = clampCfgInt(n, 1000, 1400);
}

// Load /ECUCFG.TXT (if present) and apply it, clamping every value to its valid
// range so a corrupt file can't push a limit out of bounds. Call BEFORE
// attachRpmInterrupt() so a persisted rpmedge/rpmfilter takes effect.
bool loadConfigFromSd() {
  if (!mountSd()) return false;
  if (!SD.exists(CONFIG_PATH)) { cfgFileOnSd = false; Serial.println("LOADCFG: no /ECUCFG.TXT, using defaults."); return false; }
  cfgFileOnSd = true;
  File f = SD.open(CONFIG_PATH, FILE_READ);
  if (!f) { Serial.println("LOADCFG: open FAIL, using defaults."); return false; }
  int applied = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0 || line[0] == '#') continue;
    int eq = line.indexOf('=');
    if (eq <= 0) continue;
    String key = line.substring(0, eq); key.trim();
    String val = line.substring(eq + 1); val.trim();
    applyConfigKV(key, val);
    applied++;
  }
  f.close();
  // Re-assert cross-field invariants (same as the interactive set-commands enforce).
  if (cfg.maxFuelUs < cfg.idleFuelUs) cfg.maxFuelUs = cfg.idleFuelUs;
  if (cfg.maxRpm < cfg.idleRpm + 5000) cfg.maxRpm = cfg.idleRpm + 5000;
  if (cfg.startRampToUs < cfg.startRampFromUs) cfg.startRampToUs = cfg.startRampFromUs;
  cfgLoadedFromSd = (applied > 0);
  Serial.print("LOADCFG: applied "); Serial.print(applied); Serial.println(" keys from /ECUCFG.TXT");
  if (applied > 0) addLog("CONFIG LOADED from SD");
  return cfgLoadedFromSd;
}

void initSdLogging() {
  if (!cfg.sdLoggingEnabled) {
    Serial.println("SD logging disabled by config.");
    return;
  }

  if (!mountSd()) {
    Serial.println("SD LOG: card not available. ECU continues without SD logging.");
    return;
  }

  sdWriteFailCount = 0;

  bool slotsFull = false;
  for (uint16_t i = 0; i < 1000; i++) {
    snprintf(sdLogPath, sizeof(sdLogPath), "/ECU%03u.CSV", i);
    if (!SD.exists(sdLogPath)) break;
    if (i == 999) {
      slotsFull = true;
      Serial.println("SD LOG: WARNING - all 1000 slots full! Delete old files. Appending to ECU999.CSV WITHOUT a new header to avoid corrupting its existing data.");
    }
  }

  // Only write a fresh header when sdLogPath is a brand-new (empty) file. Falling
  // back to an already-full ECU999.CSV must not insert another header row mid-file,
  // which would break any tool that assumes a single header at the top of the CSV.
  if (!slotsFull) {
    sdAppendLine("ms,type,mode,stage,egtC,dEgtCps,rpm,targetRpm,pumpUs,flowMlMin,fuelTargetUs,starterUs,ign,valve1,valve2,throttlePct,abortReason,event");
  }
  sdLogEvent(slotsFull ? "BOOT SD_LOG_READY (slots full, appending without header)" : "BOOT SD_LOG_READY");

  Serial.print("SD LOG: OK file="); Serial.println(sdLogPath);
  Serial.print("SD LOG: cardSizeMB="); Serial.println(SD.cardSize() / (1024 * 1024));
}

void addLog(const String& msg) {
  String line = String(millis() / 1000.0f, 1) + "s " + msg;
  eventLog[eventLogHead] = line;
  eventLogHead = (eventLogHead + 1) % LOG_COUNT;
  Serial.print("LOG: "); Serial.println(line);
  sdLogEvent(msg);
}

const char* testResultName(TestResult r) {
  switch (r) {
    case TEST_NOT_RUN: return "NOT_RUN";
    case TEST_RUNNING: return "RUNNING";
    case TEST_PASS: return "PASS";
    case TEST_FAIL: return "FAIL";
    default: return "UNKNOWN";
  }
}

void setChecklist(TestId id, TestResult result, const String& note) {
  if (id < 0 || id >= TEST_COUNT) return;
  checklist[id].result = result;
  checklist[id].note = note;
  checklist[id].lastMs = millis();
  addLog(String("TEST ") + checklist[id].name + " " + testResultName(result) + " - " + note);
}

void resetChecklist() {
  for (int i = 0; i < TEST_COUNT; i++) {
    checklist[i].result = TEST_NOT_RUN;
    checklist[i].note = (i == TEST_KILL) ? "Not confirmed" : "Not run";
    checklist[i].lastMs = 0;
  }
  addLog("CHECKLIST RESET");
  Serial.println("Checklist reset.");
}

bool checklistPassed(String& why) {
  for (int i = 0; i < TEST_COUNT; i++) {
    if (checklist[i].result != TEST_PASS) {
      why = String(checklist[i].name) + " - " + checklist[i].note;
      return false;
    }
  }
  why = "OK";
  return true;
}

bool dryStartRequested() {
  // Only used before beginAutoIdle(). If EGT is valid, this is a real guarded start.
  // If EGT is faulty/open and this option is enabled, startidle becomes a dry starter/RPM test.
  return cfg.allowDryStartWhenEgtFault && !egt.ok;
}

bool checklistPassedForAutoStart(String& why) {
  bool dry = dryStartRequested();
  if (dry) {
    // Dry start has no fuel, no valves and no igniter. It is a starter/RPM test,
    // so do not force the full hot-start checklist just to spin the starter.
    why = "OK_DRY_START_EGT_BYPASS";
    return true;
  }

  for (int i = 0; i < TEST_COUNT; i++) {
    if (checklist[i].result != TEST_PASS) {
      why = String(checklist[i].name) + " - " + checklist[i].note;
      return false;
    }
  }
  why = "OK";
  return true;
}

void printChecklist() {
  Serial.println("===== TEST WIZARD CHECKLIST =====");
  for (int i = 0; i < TEST_COUNT; i++) {
    Serial.print(checklist[i].name);
    Serial.print(" = ");
    Serial.print(testResultName(checklist[i].result));
    Serial.print(" | ");
    Serial.println(checklist[i].note);
  }
  String why;
  Serial.print("CHECKLIST_OVERALL=");
  Serial.println(checklistPassed(why) ? "PASS" : (String("BLOCKED: ") + why));
  Serial.print("STARTIDLE_CHECK=");
  Serial.println(checklistPassedForAutoStart(why) ? (String("PASS: ") + why) : (String("BLOCKED: ") + why));
  Serial.println("=================================");
}

bool requireArmAndIdleForTest(TestId id) {
  if (id == TEST_EGT || id == TEST_RPM_NOISE || id == TEST_KILL) return true;
  // ARM no longer required for component tests (bench convenience); still WAITING/ABORTED-only.
  if (ecuMode != MODE_WAITING && ecuMode != MODE_ABORTED) { Serial.println("ERROR: component tests only while WAITING/ABORTED."); return false; }
  return true;
}

void startTimedTest(TestId id, uint32_t durationMs) {
  activeTest = id;
  activeTestEndMs = millis() + durationMs;
  setChecklist(id, TEST_RUNNING, "Running");
}

void runTestByName(const String& nameIn) {
  String name = nameIn;
  name.trim();
  name.toLowerCase();

  updateRpm(); updateEgt();

  if (name == "egt") {
    if (egt.ok) setChecklist(TEST_EGT, TEST_PASS, String("EGT OK ") + String(egt.c, 1) + "C");
    else setChecklist(TEST_EGT, TEST_FAIL, String("EGT fault ") + egtFaultString(egt.fault));
    return;
  }

  if (name == "rpm_noise" || name == "rpm") {
    bool ok = (rpmData.noise == RPM_NO_SIGNAL || rpmData.noise == RPM_CLEAN);
    setChecklist(TEST_RPM_NOISE, ok ? TEST_PASS : TEST_FAIL,
                 String("RNOISE=") + rpmNoiseName(rpmData.noise) +
                 " raw=" + String(rpmData.rawEdges) +
                 " rej=" + String(rpmData.rejectedEdges) +
                 " jit=" + String(rpmData.jitterPct, 1) + "%");
    return;
  }

  if (name == "kill") {
    Serial.println("Kill switch cannot be electrically proven from this PCB yet. Use confirmkill after physical NC kill-switch test.");
    addLog("KILL TEST REQUESTED - use confirmkill after physical test");
    return;
  }

  TestId id = TEST_NONE;
  if (name == "ign") id = TEST_IGN;
  else if (name == "starter") id = TEST_STARTER;
  else if (name == "starter_ign" || name == "emi") id = TEST_STARTER_IGN;
  else if (name == "valve1") id = TEST_VALVE1;
  else if (name == "valve2") id = TEST_VALVE2;
  else if (name == "pump") id = TEST_PUMP;
  else { Serial.println("Unknown test. Use: test egt|rpm_noise|ign|starter|starter_ign|valve1|valve2|pump|kill"); return; }

  if (!requireArmAndIdleForTest(id)) return;

  // Igniter / fuel-valve tests must not energize into a still-hot engine (re-light
  // or fuel-into-hot-core hazard), matching the direct ignpulse / valve-on guards.
  // Starter-only test is exempt (it aids cooling); pump test has its own guard below.
  if ((id == TEST_IGN || id == TEST_STARTER_IGN || id == TEST_VALVE1 || id == TEST_VALVE2) &&
      fuelCommandBlockedByHotEgt()) {
    Serial.print("TEST BLOCKED: engine still hot (EGT="); Serial.print(egt.c, 1);
    Serial.print("C > cooldown target "); Serial.print(cfg.cooldownTargetC); Serial.println("C).");
    setChecklist(id, TEST_FAIL, "Blocked: EGT too hot");
    activeTest = TEST_NONE;
    return;
  }

  addLog(String("TEST START ") + checklist[id].name);

  switch (id) {
    case TEST_IGN:
      ignCmd = true;
      applyOutputs();
      startTimedTest(id, 1000);
      break;
    case TEST_STARTER:
      resetRpmStats();
      startUs = cfg.starterSpinUs;
      applyOutputs();
      startTimedTest(id, STARTER_TEST_SPIN_MS);
      break;
    case TEST_STARTER_IGN:
      resetRpmStats();
      startUs = cfg.starterSpinUs;
      ignCmd = true;
      applyOutputs();
      startTimedTest(id, STARTER_TEST_SPIN_MS);
      break;
    case TEST_VALVE1:
      valve1Cmd = true;
      applyOutputs();
      startTimedTest(id, 1000);
      break;
    case TEST_VALVE2:
      valve2Cmd = true;
      applyOutputs();
      startTimedTest(id, 1000);
      break;
    case TEST_PUMP:
      if (fuelCommandBlockedByHotEgt()) {
        Serial.print("PUMP TEST BLOCKED: engine still hot (EGT="); Serial.print(egt.c, 1);
        Serial.print("C > cooldown target "); Serial.print(cfg.cooldownTargetC); Serial.println("C).");
        setChecklist(TEST_PUMP, TEST_FAIL, "Blocked: EGT too hot");
        activeTest = TEST_NONE;
        break;
      }
      Serial.println("PUMP PRIME SAFETY: remove engine inlet tube and discharge fuel to container before running.");
      fuelValvesAuto(true);          // similar to ENJET oil pump test linking main fuel valve
      pumpUs = cfg.pumpTestUs;       // dedicated bench prime PWM (default 1160us, lowest measured point)
      fuelTargetUs = cfg.pumpTestUs;
      Serial.print("PUMP PRIME at "); Serial.print(cfg.pumpTestUs); Serial.print("us (~");
      Serial.print(flowFromUs(cfg.pumpTestUs), 1); Serial.println(" ml/min)");
      applyOutputs();
      startTimedTest(id, 1500);
      break;
    default:
      break;
  }
}

void updateActiveTest() {
  if (activeTest == TEST_NONE) return;
  if (millis() < activeTestEndMs) return;

  TestId done = activeTest;
  activeTest = TEST_NONE;

  switch (done) {
    case TEST_IGN:
      ignCmd = false;
      applyOutputs();
      setChecklist(done, TEST_PASS, "IGN pulse completed");
      break;
    case TEST_STARTER: {
      // Read RPM/noise WHILE the starter is still spinning. Stopping it first would
      // satisfy rpmAtRestGuardCondition() (WAITING + startStage==ST_NONE + startUs
      // back at SAFE), which forces rpmData.rpm=0 / RPM_REST_NOISE - that made the
      // verdict read RPM=0 the instant the starter stopped.
      updateRpm();
      bool ok = rpmMeasurementUsable() && rpmData.rpm >= cfg.starterProveMinRpm;
      float capturedRpm = rpmData.rpm;
      RpmNoiseLevel capturedNoise = rpmData.noise;
      startUs = ESC_SAFE_US;
      applyOutputs();
      if (ok)
        setChecklist(done, TEST_PASS, String("RPM=") + String(capturedRpm, 0) + " RNOISE=" + rpmNoiseName(capturedNoise));
      else
        setChecklist(done, TEST_FAIL, String("RPM/RNOISE bad: RPM=") + String(capturedRpm, 0) + " RNOISE=" + rpmNoiseName(capturedNoise));
      break;
    }
    case TEST_STARTER_IGN: {
      // Same as TEST_STARTER: capture the noise verdict while spinning, before the
      // rest-guard zeroes it.
      updateRpm();
      RpmNoiseLevel capturedNoise = rpmData.noise;
      startUs = ESC_SAFE_US;
      ignCmd = false;
      applyOutputs();
      if (capturedNoise == RPM_NO_SIGNAL || capturedNoise == RPM_CLEAN)
        setChecklist(done, TEST_PASS, String("EMI OK RNOISE=") + rpmNoiseName(capturedNoise));
      else
        setChecklist(done, TEST_FAIL, String("EMI/RPM noisy RNOISE=") + rpmNoiseName(capturedNoise));
      break;
    }
    case TEST_VALVE1:
      valve1Cmd = false;
      applyOutputs();
      setChecklist(done, TEST_PASS, "Valve 1 pulse completed");
      break;
    case TEST_VALVE2:
      valve2Cmd = false;
      applyOutputs();
      setChecklist(done, TEST_PASS, "Valve 2 pulse completed");
      break;
    case TEST_PUMP:
      pumpUs = ESC_SAFE_US;
      fuelTargetUs = ESC_SAFE_US;
      fuelValvesAuto(false);
      applyOutputs();
      setChecklist(done, TEST_PASS, String("Pump prime completed at ") + String(cfg.pumpTestUs) + "us");
      break;
    default:
      break;
  }
}

String jsonEscape(const String& in) {
  String out;
  out.reserve(in.length() + 8);
  for (uint16_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '\\') out += "\\\\";
    else if (c == '"') out += "\\\"";
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if (c == '\t') out += "\\t";
    else if ((uint8_t)c < 0x20) {
      // Any other control char would produce invalid JSON; emit \u00XX.
      static const char hex[] = "0123456789abcdef";
      out += "\\u00"; out += hex[((uint8_t)c >> 4) & 0xF]; out += hex[(uint8_t)c & 0xF];
    }
    else out += c;
  }
  return out;
}

String logsJoined() {
  String s;
  for (uint8_t i = 0; i < LOG_COUNT; i++) {
    uint8_t idx = (eventLogHead + i) % LOG_COUNT;
    if (eventLog[idx].length()) {
      if (s.length()) s += "<br>";
      s += jsonEscape(eventLog[idx]);
    }
  }
  return s;
}

String checklistJson() {
  String s = "[";
  for (int i = 0; i < TEST_COUNT; i++) {
    if (i) s += ",";
    s += "{\"name\":\"" + String(checklist[i].name) + "\",\"result\":\"" + String(testResultName(checklist[i].result)) + "\",\"note\":\"" + jsonEscape(checklist[i].note) + "\"}";
  }
  s += "]";
  return s;
}

String webStatusJson() {
  updateRpm(); updateEgt();
  String ckWhy;
  bool ckOk = checklistPassedForAutoStart(ckWhy);
  String s;
  s.reserve(5120);  // pre-size for the worst-case payload (status + checklist + 16 long logs + full cfg) to avoid heap churn per /api poll
  s = "{";
  s += "\"mode\":\"" + String(modeName(ecuMode)) + "\",";
  s += "\"stage\":\"" + String(stageName(startStage)) + "\",";
  if (egt.ok) s += "\"egt\":\"" + String(egt.c, 1) + " C\",";
  else s += "\"egt\":\"ERR " + jsonEscape(egtFaultString(egt.fault)) + "\",";
  s += "\"degt\":\"" + String(egt.gradientCps, 1) + " C/s\",";
  s += "\"rpm\":\"" + String(rpmData.rpm, 0) + "\",";
  // Numeric values (unquoted) for the SVG gauges.
  s += "\"rpmv\":" + String(rpmData.rpm, 0) + ",";
  s += "\"egtv\":" + String(egt.ok ? egt.c : 0.0f, 0) + ",";
  s += "\"rtgt\":\"" + String(fuelTargetRpm) + "\",";
  s += "\"rnoise\":\"" + String(rpmNoiseName(rpmData.noise)) + "\",";
  s += "\"rpmDetail\":\"raw=" + String(rpmData.rawEdges) + " acc=" + String(rpmData.acceptedWindow) + " rej=" + String(rpmData.rejectedEdges) + " jit=" + String(rpmData.jitterPct, 1) + "%" + (rpmData.restPulseNoise ? " REST_GUARD" : "") + "\",";
  s += "\"pump\":\"" + String(pumpUs) + " us / " + String(flowFromUs(pumpUs), 1) + " ml/min\",";
  s += "\"ftgt\":\"" + String(fuelTargetUs) + " us / " + String(flowFromUs(fuelTargetUs), 1) + " ml/min\",";
  s += "\"start\":\"" + String(startUs) + " us\",";
  s += "\"ign\":\"" + String(ignCmd ? "ON" : "OFF") + "\",";
  s += "\"v1\":\"" + String(valve1Cmd ? "ON" : "OFF") + "\",";
  s += "\"v2\":\"" + String(valve2Cmd ? "ON" : "OFF") + "\",";
  s += "\"manAuto\":\"" + String(manAutoStatusText()) + "\",";
  s += "\"thr\":\"" + String(throttlePct) + "%\",";
  s += "\"arm\":\"" + String(isStage2Armed() ? "ARMED" : "LOCKED") + "\",";
  s += "\"auto\":\"" + String(cfg.autoStartEnabled ? "ON" : "OFF") + "\",";
  s += "\"abort\":\"" + jsonEscape(lastAbortReason) + "\",";
  s += "\"checklistOk\":\"" + String(ckOk ? "PASS" : "BLOCKED") + "\",";
  s += "\"checklistWhy\":\"" + jsonEscape(ckWhy) + "\",";
  s += "\"sd\":\"" + String(sdOk ? (cfg.sdLoggingEnabled ? String("OK ") + sdLogPath : "OFF") : "FAIL") + "\",";
  s += "\"checklist\":" + checklistJson() + ",";
  // Live config values so the Web UI tune inputs always reflect the actual config
  // (prevents stale literal defaults from silently changing limits when Set is clicked).
  s += "\"cfgIdleRpm\":\"" + String(cfg.idleRpm) + "\",";
  s += "\"cfgMaxRpm\":\"" + String(cfg.maxRpm) + "\",";
  s += "\"cfgMaxEgt\":\"" + String(cfg.maxEgtC) + "\",";
  s += "\"cfgPurgeUs\":\"" + String(cfg.starterPurgeUs) + "\",";
  s += "\"cfgSpinUs\":\"" + String(cfg.starterSpinUs) + "\",";
  s += "\"cfgAssistUs\":\"" + String(cfg.starterAssistUs) + "\",";
  s += "\"cfgIntroUs\":\"" + String(cfg.introFuelUs) + "\",";
  s += "\"cfgIdleUs\":\"" + String(cfg.idleFuelUs) + "\",";
  s += "\"cfgMaxUs\":\"" + String(cfg.maxFuelUs) + "\",";
  s += "\"cfgPumpTestUs\":\"" + String(cfg.pumpTestUs) + "\",";
  // Start-sequence params
  s += "\"cfgRampFromUs\":\"" + String(cfg.startRampFromUs) + "\",";
  s += "\"cfgRampToUs\":\"" + String(cfg.startRampToUs) + "\",";
  s += "\"cfgRampStepMs\":\"" + String(cfg.startRampStepMs) + "\",";
  s += "\"cfgEscArmHoldMs\":\"" + String(cfg.escArmHoldMs) + "\",";
  s += "\"cfgPurgeStableMs\":\"" + String(cfg.purgeStableMs) + "\",";
  s += "\"cfgIgnArmRpm\":\"" + String(cfg.ignArmRpm) + "\",";
  s += "\"cfgSpinupTimeoutMs\":\"" + String(cfg.spinupRpmTimeoutMs) + "\",";
  s += "\"cfgFuelDelayMs\":\"" + String(cfg.fuelDelayMs) + "\",";
  s += "\"cfgLightOffRise\":\"" + String(cfg.lightOffMinRiseCps) + "\",";
  s += "\"cfgLightOffConfirmMs\":\"" + String(cfg.lightOffConfirmMs) + "\",";
  s += "\"cfgFlameProveMs\":\"" + String(cfg.flameProveMs) + "\",";
  s += "\"cfgFlameOutDropC\":\"" + String(cfg.flameOutDropC) + "\",";
  s += "\"cfgAccelHoldMs\":\"" + String(cfg.accelHoldMs) + "\",";
  s += "\"cfgAccelStepUs\":\"" + String(cfg.accelStepUs) + "\",";
  s += "\"cfgPurgeOutMs\":\"" + String(cfg.purgeOutMs) + "\",";
  s += "\"cfgStarterMaxRpm\":\"" + String(cfg.starterMaxRpm) + "\",";
  s += "\"cfgHotSpinMinRpm\":\"" + String(cfg.hotSpinMinRpm) + "\",";
  s += "\"cfgHotSpinUs\":\"" + String(cfg.hotSpinUs) + "\",";
  // RPM sensor + safety + timing config, so every Web UI tune input auto-populates
  // from the live config (no terminal needed).
  s += "\"cfgRpmTol\":\"" + String(cfg.rpmTolerance) + "\",";
  s += "\"cfgPpr\":\"" + String(cfg.pulsesPerRev) + "\",";
  s += "\"cfgRpmFilter\":\"" + String((uint32_t)rpmMinPulseUs) + "\",";
  s += "\"cfgRpmEdge\":\"" + String(rpmEdgeName()) + "\",";
  s += "\"cfgMaxGrad\":\"" + String(cfg.maxTempGradientCps) + "\",";
  s += "\"cfgDryStartMs\":\"" + String(cfg.dryStartRunMs) + "\",";
  s += "\"cfgAccelMs\":\"" + String(cfg.accelStepDelayMs) + "\",";
  s += "\"cfgDecelMs\":\"" + String(cfg.decelStepDelayMs) + "\",";
  s += "\"cfgLowAccelMs\":\"" + String(cfg.lowAccelStepDelayMs) + "\",";
  s += "\"cfgLowDecelMs\":\"" + String(cfg.lowDecelStepDelayMs) + "\",";
  s += "\"cfgAccelToIdleMs\":\"" + String(cfg.accelToIdleTimeoutMs) + "\",";
  s += "\"cfgCoolTarget\":\"" + String(cfg.cooldownTargetC) + "\",";
  s += "\"cfgCoolStarter\":\"" + String(cfg.cooldownStarterUs) + "\",";
  s += "\"cfgCoolMinMs\":\"" + String(cfg.cooldownMinMs) + "\",";
  s += "\"cfgCoolTimeoutMs\":\"" + String(cfg.cooldownTimeoutMs) + "\",";
  s += "\"cfgCommTimeout\":\"" + String(cfg.commTimeoutMs) + "\",";
  // Interlock/toggle states so the Web UI can show and flip them.
  s += "\"swChecklist\":\"" + String(cfg.requireChecklistForStart ? "ON" : "OFF") + "\",";
  s += "\"swCommWd\":\"" + String(cfg.commWatchdogEnabled ? "ON" : "OFF") + "\",";
  s += "\"swSdlog\":\"" + String(cfg.sdLoggingEnabled ? "ON" : "OFF") + "\",";
  s += "\"swEgtDry\":\"" + String(cfg.allowDryStartWhenEgtFault ? "DRY" : "STRICT") + "\",";
  s += "\"cfgOnSd\":\"" + String(cfgFileOnSd ? "YES" : "NO") + "\",";
  s += "\"cfgLoaded\":\"" + String(cfgLoadedFromSd ? "YES" : "NO") + "\",";
  s += "\"logs\":\"" + logsJoined() + "\"";
  s += "}";
  return s;
}


void setupWebServer() {
  if (webRoutesReady) return;
  // The ECU no longer serves its own dashboard HTML - the UI now lives in the PC-side
  // SerialWebBridge tool, which talks to this firmware over Serial OR over WiFi via the
  // machine-facing /api and /cmd endpoints below. "/" just returns a short pointer so a
  // stray browser hitting http://192.168.4.1 isn't left with a blank/404 page.
  server.on("/", []() {
    server.send(200, "text/plain; charset=utf-8",
      "ECU Test V1 - JSON API only (no built-in web UI).\n"
      "Dung cong cu SerialWebBridge (ket noi qua Day/Serial hoac WiFi) de dieu khien.\n"
      "Endpoints: GET /api (trang thai JSON), GET /cmd?c=<lenh>.\n");
  });
  server.on("/api", []() {
    // Only a poll from a VISIBLE dashboard tab counts as operator presence for the
    // comm watchdog. The page sends act=0 when hidden/backgrounded (phone locked,
    // tab switched) so a walked-away session times out instead of being kept alive
    // forever by a background poll. Missing arg (old cached page) = treat as active.
    if (server.arg("act") != "0") lastOperatorLinkMs = millis();
    server.send(200, "application/json", webStatusJson());
  });
  server.on("/cmd", []() {
    String c = server.hasArg("c") ? server.arg("c") : "";
    if (c.length()) handleCommand(c);
    server.send(200, "text/plain", "OK");
  });
  webRoutesReady = true;
}

void startWebServer() {
  cfg.webEnabled = true;
  setupWebServer();
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(webIp, webGateway, webSubnet);
  WiFi.softAP(WEB_SSID, WEB_PASS);
  server.begin();
  webStarted = true;
  addLog(String("WIFI API ON ") + WEB_SSID + " http://192.168.4.1");
  Serial.print("WiFi JSON API: SSID="); Serial.print(WEB_SSID); Serial.println(" PASS=admin1234 URL=http://192.168.4.1 (/api,/cmd) - dung SerialWebBridge lam UI");
}

void stopWebServer() {
  cfg.webEnabled = false;
  webStarted = false;
  WiFi.softAPdisconnect(true);
  addLog("WEB OFF");
  Serial.println("Web UI OFF.");
}

bool canStartAutoIdle(String& why) {
  updateRpm();
  updateEgt();

  if (rpmCalMode) { why = "RPM_CAL_MODE still ON (rest-guard disabled) - type 'set rpmcal off' first"; return false; }
  if (!cfg.autoStartEnabled) { why = "auto-start disabled; use arm2 then autostart on"; return false; }
  if (ecuMode != MODE_WAITING && ecuMode != MODE_ABORTED) { why = "ECU not in WAITING/ABORTED"; return false; }
  if (ecuMode == MODE_ABORTED && !abortAcknowledged) {
    why = String("ABORT not acknowledged (reason: ") + lastAbortReason + "). Type clearabort first.";
    return false;
  }

  bool dry = dryStartRequested();
  if (cfg.requireEgtForStart && !egt.ok && !dry) {
    why = "EGT fault / thermocouple not ready";
    return false;
  }
  if (egt.ok && egt.c > (cfg.ignitionThresholdC - 10)) { why = "EGT still too hot for restart"; return false; }
  if (cfg.maxFuelUs < cfg.idleFuelUs) { why = "maxFuelUs is below idleFuelUs"; return false; }
  if (cfg.requireRpmForStart && rpmNoiseBlocksStart()) {
    why = String("RPM_NOT_STABLE_AT_REST: ") + rpmNoiseName(rpmData.noise) +
          " raw=" + String(rpmData.rawEdges) + " acc=" + String(rpmData.acceptedWindow);
    return false;
  }
  if (cfg.requireChecklistForStart) {
    String ck;
    if (!checklistPassedForAutoStart(ck)) {
      why = String("CHECKLIST_NOT_PASSED: ") + ck;
      return false;
    }
  }
  if (dry) {
    why = String("EGT_FAULT_DRY_START_ONLY: ") + egtFaultString(egt.fault);
  } else {
    why = "OK";
  }
  return true;
}

void beginAutoIdle() {
  dryStartActive = dryStartRequested();
  cooldownAfterAbort = false;
  // Cancel any component test still counting down so updateActiveTest() cannot fire
  // mid-sequence and stall the starter or valves during the real start.
  activeTest = TEST_NONE; activeTestEndMs = 0;
  ignitionDetectedMs = 0;
  throttlePct = 0; lastAbortReason = "NONE"; starterAboveReleaseSinceMs = 0; manualIgnOffAtMs = manualStartOffAtMs = manualPumpOffAtMs = 0;
  // Fresh comm-watchdog window for the new run; assume a remote (Serial/Web) start
  // unless startIdleFromButton() flips this to a button run right after.
  lastOperatorLinkMs = millis();
  runStartedByButton = false;
  fuelTargetUs = ESC_SAFE_US;
  startRampBeganMs = millis(); lightoffPhase = 0; lightoffPhaseMs = 0; flameRefEgtC = 0.0f;
  accelStepAtMs = 0; accelLastRpm = 0.0f;
  resetRpmStats();   // clean RPM window before the purge ramp measures the ign-arm crossing
  forceSafeOutputs(); enterMode(MODE_STARTING); enterStage(ST_PURGE);
  startUs = cfg.startRampFromUs; pumpUs = ESC_SAFE_US; ignCmd = false; fuelValvesAuto(false); applyOutputs();

  if (dryStartActive) {
    addLog("DRY START: EGT fault, fuel/valves/ign OFF");
    Serial.print("DRY START/RPM TEST: EGT fault (");
    Serial.print(egtFaultString(egt.fault));
    Serial.println("). PUMP, VALVES and IGN are forced OFF. This will NOT start the engine.");
  } else {
    addLog("AUTO-IDLE START");
    Serial.println("AUTO-IDLE START: entering PURGE (starter ramp from 1150us until RPM>ignArmRpm)");
  }
}

// PURGE starter PWM, 3 phases (see cfg.escArmHoldMs comment): ESC arm hold at
// ESC_SAFE_US, then steady hold at startRampFromUs, then ramp +1us per
// startRampStepMs capped at startRampToUs. Shared by the dry test and a real start.
int starterRampUs() {
  uint32_t el = millis() - startRampBeganMs;
  if (el <= cfg.escArmHoldMs) return ESC_SAFE_US;
  uint32_t afterArm = el - cfg.escArmHoldMs;
  if (afterArm <= cfg.purgeStableMs) return cfg.startRampFromUs;
  uint32_t rampEl = afterArm - cfg.purgeStableMs;
  uint32_t step = (cfg.startRampStepMs > 0) ? cfg.startRampStepMs : 1;
  long add = (long)(rampEl / step);
  long us = (long)cfg.startRampFromUs + add;
  if (us > cfg.startRampToUs) us = cfg.startRampToUs;
  return (int)us;
}

// Starter-prove-RPM check anchor: the STARTER_PROVE_TIMEOUT_MS clock should only
// start once the ESC arm-hold phase ends (before that, the starter is deliberately
// held at zero throttle, so RPM==0 is expected, not a starter fault).
uint32_t starterProveDeadlineMs() { return stageEnteredMs + cfg.escArmHoldMs + STARTER_PROVE_TIMEOUT_MS; }

// Starter-assist PWM tracking RPM: linear starterAssistUs (min) -> startRampToUs (max)
// as RPM goes 0 -> starterMaxRpm. Returns ESC_SAFE_US (starter released) at/above
// starterMaxRpm.
int assistUsForRpm(float rpm) {
  if (rpm >= (float)cfg.starterMaxRpm) return ESC_SAFE_US;
  if (rpm < 0.0f) rpm = 0.0f;
  long span = (long)cfg.startRampToUs - (long)cfg.starterAssistUs;
  long us = (long)cfg.starterAssistUs + (long)(span * rpm / (float)cfg.starterMaxRpm);
  if (us < cfg.starterAssistUs) us = cfg.starterAssistUs;
  if (us > cfg.startRampToUs) us = cfg.startRampToUs;
  return (int)us;
}

void updateDryStarting() {
  // Dry starter/RPM test used only when EGT is OPEN/faulty.
  // Fuel, valves and igniter are deliberately forced OFF on every loop.
  fuelTargetRpm = 0;
  fuelTargetUs = ESC_SAFE_US;
  pumpUs = ESC_SAFE_US;
  ignCmd = false;
  fuelValvesAuto(false);

  switch (startStage) {
    case ST_PURGE:
      startUs = starterRampUs();
      if (cfg.requireRpmForStart && millis() > starterProveDeadlineMs() &&
          (!rpmSignalRecentWithin(RPM_SIGNAL_TIMEOUT_MS) || rpmData.rpm < cfg.starterProveMinRpm)) {
        abortAll("DRY_NO_STARTER_RPM");
        return;
      }
      if (rpmMeasurementUsable() && rpmData.rpm >= (float)cfg.ignArmRpm) {
        enterStage(ST_ACCEL_TO_IDLE);
        Serial.println("DRY START STAGE -> RUN_STARTER (RPM up)");
      } else if (millis() - startRampBeganMs >= cfg.spinupRpmTimeoutMs) {
        abortAll("DRY_NO_RPM");
        return;
      }
      break;

    case ST_ACCEL_TO_IDLE:
      startUs = cfg.startRampToUs;
      if (millis() - stageEnteredMs >= cfg.dryStartRunMs) {
        Serial.print("DRY START COMPLETE -> WAITING. RPM=");
        Serial.print(rpmData.rpm, 0);
        Serial.print(" RNOISE=");
        Serial.println(rpmNoiseName(rpmData.noise));
        enterWaitingSafe();
      }
      break;

    default:
      abortAll("INVALID_DRY_START_STAGE");
      return;
  }
}

bool checkPostIgnitionRpmRise() {
  // After light-off, RPM must climb past fuelConfirmRpm within fuelConfirmTimeoutMs.
  // Anchored on ignitionDetectedMs so it spans PREHEAT_FUEL and ACCEL_TO_IDLE.
  // Returns true after aborting.
  if (!cfg.requireRpmForStart) return false;
  if (ignitionDetectedMs == 0) return false;
  if (millis() - ignitionDetectedMs < cfg.fuelConfirmTimeoutMs) return false;
  if (rpmMeasurementUsable() && rpmData.rpm >= cfg.fuelConfirmRpm) return false;
  abortAll("NO_RPM_RISE");
  return true;
}

// Flame-out test: lost if EGT falls below the ignition threshold, or drops more than
// flameOutDropC within ~2s (rolling reference), or the thermocouple reads faulty.
bool flameLost() {
  if (!egt.ok) return true;
  if (egt.c < (float)cfg.ignitionThresholdC) return true;
  uint32_t now = millis();
  if (flameRefMs == 0 || now - flameRefMs >= 2000) { flameRefEgtC = egt.c; flameRefMs = now; }
  if (egt.c < flameRefEgtC - (float)cfg.flameOutDropC) return true;
  return false;
}

void manAutoFail(const char* reason) {
  // Cuts glow/fuel/valves only - deliberately does NOT touch startUs, same as the real
  // startFailed(): the starter must keep running (per spec, stage-1 rule RPM>3000) to
  // blow unburned fuel out, not shut off with everything else. purgeOutUntilMs lets the
  // shared applyStarterProtection() hold/restore RPM through the blow-out window if needed.
  ignCmd = false; pumpUs = ESC_SAFE_US; fuelTargetUs = ESC_SAFE_US; fuelValvesAuto(false);
  purgeOutUntilMs = millis() + cfg.purgeOutMs;
  manAutoActive = false; applyOutputs();
  Serial.print("MANUAL AUTO START FAILED: "); Serial.print(reason); Serial.println(" - starter left running (blow-out), turn off manually when done.");
}

// Manual page "AUTO START" sequence - mirrors ST_LIGHTOFF's phase 0/1/2 logic exactly
// (see updateStarting()'s ST_LIGHTOFF case), reusing the same tunables, but without any
// starter/RPM/ACCEL_TO_IDLE involvement: valve1+pump+glow are already on (set by the
// "manauto on" command) by the time this runs each loop.
void updateManAuto() {
  if (!manAutoActive) return;
  // checkFailures() (OVER_TEMP/OVERSPEED/EGT_FAULT) returns immediately while
  // ecuMode is WAITING/ABORTED - exactly the mode this sequence runs in - so it
  // provides no protection here. Guard OVER_TEMP explicitly, same condition/
  // look-ahead as checkFailures(), since real fuel/glow are active in phases 1-2.
  if (egt.ok && (egt.c >= (float)cfg.maxEgtC ||
      (egt.gradientCps > 0.0f && (egt.c + EGT_ABORT_LOOKAHEAD_S * egt.gradientCps) >= (float)cfg.maxEgtC))) {
    manAutoFail("OVER_TEMP"); return;
  }
  uint32_t now = millis();
  if (manAutoPhase == 0) {
    valve1Cmd = true; valve2Cmd = false;
    if (now - manAutoPhaseMs >= cfg.fuelDelayMs) {
      bool rising = egt.ok && egt.c >= (float)cfg.ignitionThresholdC &&
                    egt.gradientCps >= (float)cfg.lightOffMinRiseCps;
      if (rising) {
        if (manAutoRiseSinceMs == 0) manAutoRiseSinceMs = now;
      } else {
        manAutoRiseSinceMs = 0; // condition broke, restart the hold timer
      }
      if (manAutoRiseSinceMs != 0 && now - manAutoRiseSinceMs >= cfg.lightOffConfirmMs) {
        valve2Cmd = true; // confirmed light-off -> open Main Valve
        flameRefEgtC = egt.c; flameRefMs = now;
        manAutoPhase = 1; manAutoPhaseMs = now;
        Serial.print("MANUAL AUTO START: light-off confirmed (EGT="); Serial.print(egt.c, 0);
        Serial.print("C, dEGT="); Serial.print(egt.gradientCps, 0); Serial.println("C/s) -> open MAIN valve");
      } else if (now - manAutoStartedMs >= cfg.noIgnitionTimeoutMs) {
        manAutoFail("NO_IGNITION"); return;
      }
    }
  } else if (manAutoPhase == 1) {
    valve1Cmd = true; valve2Cmd = true;
    if (flameLost()) { manAutoFail("FLAME_LOST"); return; }
    if (now - manAutoPhaseMs >= cfg.postIgnitionHeatMs) {
      valve1Cmd = false; // close Start Valve
      manAutoPhase = 2; manAutoPhaseMs = now;
      Serial.println("MANUAL AUTO START: close START valve, prove flame");
    }
  } else {
    valve1Cmd = false; valve2Cmd = true;
    if (flameLost()) { manAutoFail("FLAME_LOST"); return; }
    if (now - manAutoPhaseMs >= cfg.flameProveMs) {
      ignCmd = false; manAutoActive = false;
      Serial.println("MANUAL AUTO START OK: glow OFF, pump/valve2 stay under manual control.");
    }
  }
  applyOutputs();
}

String manAutoStatusText() {
  if (!manAutoActive) return "OFF";
  if (manAutoPhase == 0) return "Chờ bắt lửa thật (EGT/dEGT)";
  if (manAutoPhase == 1) return "Đã bắt lửa - Valve2 mở, chờ ổn định";
  return "Đóng Valve1 - đang xác nhận lửa còn cháy";
}

// Failed light-off: cut fuel/glow/valves, keep the starter blowing unburned fuel out
// for purgeOutMs (via applyStarterProtection), then abort to cooldown.
void startFailed(const char* reason) {
  Serial.print("START FAILED: "); Serial.println(reason);
  ignCmd = false; pumpUs = ESC_SAFE_US; fuelTargetUs = ESC_SAFE_US; valve1Cmd = false; valve2Cmd = false;
  purgeOutUntilMs = millis() + cfg.purgeOutMs;
  abortAll(reason);
}

void updateStarting() {
  if (dryStartActive) {
    updateDryStarting();
    return;
  }
  switch (startStage) {
    case ST_PURGE: {
      // Starter ramps from 1150us (+1us/step) with no fuel/glow. Once RPM > ignArmRpm,
      // dwell purgeTimeMs to blow residual gas/fuel out, then preheat.
      startUs = starterRampUs(); pumpUs = ESC_SAFE_US; ignCmd = false; valve1Cmd = false; valve2Cmd = false;
      if (cfg.requireRpmForStart && millis() > starterProveDeadlineMs() && (!rpmSignalRecentWithin(RPM_SIGNAL_TIMEOUT_MS) || rpmData.rpm < cfg.starterProveMinRpm)) { abortAll("NO_STARTER_RPM"); return; }
      if (rpmMeasurementUsable() && rpmData.rpm >= (float)cfg.ignArmRpm) {
        if (starterAboveReleaseSinceMs == 0) starterAboveReleaseSinceMs = millis();      // start purge dwell
        if (millis() - starterAboveReleaseSinceMs >= cfg.purgeTimeMs) {
          starterAboveReleaseSinceMs = 0; enterStage(ST_SPINUP_PREHEAT); Serial.println("START -> SPINUP_PREHEAT");
          return;
        }
      } else {
        starterAboveReleaseSinceMs = 0;                                                  // fell below arm RPM, reset dwell
        if (millis() - startRampBeganMs >= cfg.spinupRpmTimeoutMs) { abortAll("NO_RPM"); return; }
      }
      break;
    }
    case ST_SPINUP_PREHEAT:
      // Glow preheat; starter assist tracks RPM. Glow stays ON after preheatMs.
      startUs = assistUsForRpm(rpmData.rpm); pumpUs = ESC_SAFE_US; ignCmd = true; valve1Cmd = false; valve2Cmd = false;
      if (millis() - stageEnteredMs >= cfg.preheatMs) { enterStage(ST_INTRO_FUEL); Serial.println("START -> INTRO_FUEL"); }
      break;
    case ST_INTRO_FUEL:
      // Glow ON, open Start Valve (valve1) only, begin fuel at introFuelUs (min).
      startUs = assistUsForRpm(rpmData.rpm); ignCmd = true; valve1Cmd = true; valve2Cmd = false;
      setFuelTargetUs(cfg.introFuelUs); stepPumpToward(fuelTargetUs, cfg.lowAccelStepDelayMs, cfg.lowDecelStepDelayMs);
      lightoffPhase = 0; lightoffPhaseMs = millis(); flameRefMs = 0; lightoffRiseSinceMs = 0;
      enterStage(ST_LIGHTOFF); Serial.println("START -> LIGHTOFF");
      break;
    case ST_LIGHTOFF: {
      startUs = assistUsForRpm(rpmData.rpm); ignCmd = true;
      setFuelTargetUs(max(fuelTargetUs, cfg.introFuelUs)); stepPumpToward(fuelTargetUs, cfg.lowAccelStepDelayMs, cfg.lowDecelStepDelayMs);
      uint32_t now = millis();
      if (lightoffPhase == 0) {
        // Start valve open, wait for fuel to reach chamber, then confirm REAL light-off:
        // EGT >= threshold AND rising >= lightOffMinRiseCps, held for lightOffConfirmMs.
        // (Guards against glow alone pushing EGT past the threshold without combustion.)
        valve1Cmd = true; valve2Cmd = false;
        if (now - lightoffPhaseMs >= cfg.fuelDelayMs) {
          bool rising = egt.ok && egt.c >= (float)cfg.ignitionThresholdC &&
                        egt.gradientCps >= (float)cfg.lightOffMinRiseCps;
          if (rising) {
            if (lightoffRiseSinceMs == 0) lightoffRiseSinceMs = now;
          } else {
            lightoffRiseSinceMs = 0;                    // condition broke, restart the hold timer
          }
          if (lightoffRiseSinceMs != 0 && now - lightoffRiseSinceMs >= cfg.lightOffConfirmMs) {
            valve2Cmd = true;                          // confirmed light-off -> open Main Valve
            flameRefEgtC = egt.c; flameRefMs = now;
            lightoffPhase = 1; lightoffPhaseMs = now;
            Serial.print("LIGHTOFF confirmed (EGT="); Serial.print(egt.c, 0); Serial.print("C, dEGT="); Serial.print(egt.gradientCps, 0); Serial.println("C/s) -> open MAIN valve");
          } else if (now - stageEnteredMs >= cfg.noIgnitionTimeoutMs) {
            startFailed("NO_IGNITION"); return;
          }
        }
      } else if (lightoffPhase == 1) {
        // Both valves open; let flame stabilize postIgnitionHeatMs, then close Start Valve.
        valve1Cmd = true; valve2Cmd = true;
        if (flameLost()) { startFailed("FLAME_LOST"); return; }
        if (now - lightoffPhaseMs >= cfg.postIgnitionHeatMs) {
          valve1Cmd = false;                           // close Start Valve
          lightoffPhase = 2; lightoffPhaseMs = now;
          Serial.println("LIGHTOFF: close START valve, prove flame");
        }
      } else {
        // Main valve only; prove flame self-sustains for flameProveMs, then glow OFF.
        valve1Cmd = false; valve2Cmd = true;
        if (flameLost()) { startFailed("FLAME_LOST"); return; }
        if (now - lightoffPhaseMs >= cfg.flameProveMs) {
          ignCmd = false; ignitionDetectedMs = now;
          accelStepAtMs = now; accelLastRpm = rpmMeasurementUsable() ? rpmData.rpm : 0.0f;
          enterStage(ST_ACCEL_TO_IDLE); Serial.println("LIGHTOFF OK -> ACCEL_TO_IDLE (glow off)");
        }
      }
      break;
    }
    case ST_ACCEL_TO_IDLE: {
      // Main valve only, glow off. Starter assist tracks RPM and releases at starterMaxRpm.
      ignCmd = false; valve1Cmd = false; valve2Cmd = true;
      startUs = assistUsForRpm(rpmData.rpm);
      if (flameLost()) { startFailed("FLAME_LOST"); return; }
      if (millis() - stageEnteredMs >= cfg.accelToIdleTimeoutMs) { abortAll("ACCEL_TO_IDLE_TIMEOUT"); return; }
      if (rpmMeasurementUsable() && rpmData.rpm >= (float)cfg.idleRpm) {
        enterMode(MODE_IDLING); enterStage(ST_NONE); startUs = ESC_SAFE_US; fuelTargetUs = pumpUs;
        Serial.println("START COMPLETE -> IDLING"); return;
      }
      // Bump fuel slowly only when RPM is not rising (flame alive but stalled).
      uint32_t now = millis();
      if (now - accelStepAtMs >= cfg.accelHoldMs) {
        float rpmNow = rpmMeasurementUsable() ? rpmData.rpm : 0.0f;
        if (rpmNow <= accelLastRpm + 50.0f) setFuelTargetUs(min(fuelTargetUs + cfg.accelStepUs, (int)cfg.maxFuelUs));
        accelLastRpm = rpmNow; accelStepAtMs = now;
      }
      stepPumpToward(fuelTargetUs, cfg.lowAccelStepDelayMs, cfg.lowDecelStepDelayMs);
      break;
    }
    default: abortAll("INVALID_START_STAGE"); return;
  }
}

void updateIdling() {
  fuelValvesAuto(true); ignCmd = false; startUs = ESC_SAFE_US;
  updateFuelClosedLoopToRpm(cfg.idleRpm, ESC_SAFE_US, cfg.maxFuelUs, true);
  if (throttlePct > 0) { enterMode(MODE_OPERATING); fuelTargetUs = pumpUs; Serial.println("IDLING -> OPERATING"); }
}

void updateOperating() {
  fuelValvesAuto(true); ignCmd = false; startUs = ESC_SAFE_US;
  // Governed operating ceiling sits one rpmTolerance BELOW maxRpm so that 100%
  // throttle does not target the redline itself and trip its own OVERSPEED abort
  // (which fires at rpm >= maxRpm) on any overshoot/noise.
  int operatingMaxRpm = cfg.maxRpm - cfg.rpmTolerance;
  if (operatingMaxRpm < cfg.idleRpm) operatingMaxRpm = cfg.idleRpm;
  int targetRpm = cfg.idleRpm + (int)lroundf(((float)(operatingMaxRpm - cfg.idleRpm) * (float)throttlePct) / 100.0f);
  updateFuelClosedLoopToRpm(targetRpm, ESC_SAFE_US, cfg.maxFuelUs, false);
  if (throttlePct <= 0) { enterMode(MODE_IDLING); fuelTargetUs = pumpUs; Serial.println("OPERATING -> IDLING"); }
}

void updateCooldown() {
  // Real cooldown: fuel path and igniter are always OFF, starter gently moves air.
  // Finish after minimum cooldown AND either EGT is low enough or timeout expires.
  fuelTargetRpm = 0;
  fuelTargetUs = ESC_SAFE_US;
  pumpUs = ESC_SAFE_US;
  ignCmd = false;
  fuelValvesAuto(false);

  uint32_t elapsedMs = millis() - modeEnteredMs;
  bool minDone = elapsedMs >= cfg.cooldownMinMs;
  bool tempLow = egt.ok && egt.c <= cfg.cooldownTargetC;
  bool timedOut = elapsedMs >= cfg.cooldownTimeoutMs;

  if (minDone && (tempLow || timedOut)) {
    startUs = ESC_SAFE_US;
    fuelTargetUs = ESC_SAFE_US;
    pumpUs = ESC_SAFE_US;
    ignCmd = false;
    fuelValvesAuto(false);
    applyOutputs();

    if (cooldownAfterAbort) {
      cooldownAfterAbort = false;
      enterMode(MODE_ABORTED);
      enterStage(ST_NONE);
      Serial.println("COOLDOWN -> ABORTED. Inspect fault, fix cause, then clear only when safe.");
      addLog("COOLDOWN COMPLETE -> ABORTED");
    } else {
      enterWaitingSafe();
      Serial.println("COOLDOWN -> WAITING");
    }
    return;
  }

  startUs = constrain(cfg.cooldownStarterUs, ESC_SAFE_US, ESC_MAX_US);
}

// Engine/starter protection, evaluated every loop AFTER the mode logic sets startUs
// and BEFORE applyOutputs(), so it authoritatively overrides the per-mode starter
// command in every state (starting, running, cooldown, aborted, waiting).
void applyStarterProtection() {
  bool rpmOk = rpmMeasurementUsable();
  // (2) Starter overspeed: above starterMaxRpm force the starter OFF so the spun-up
  //     impeller cannot overload/over-spin the starter motor. Wins over the hot guard.
  if (rpmOk && rpmData.rpm >= (float)cfg.starterMaxRpm) { startUs = ESC_SAFE_US; hotSpinCurUs = 0; return; }
  // (1) Hot soak-back guard (+ post-fail purge): while EGT is above the hot threshold
  //     (cooldownTargetC) OR we are inside the post-fail purge window, and RPM is below
  //     hotSpinMinRpm, spin the starter - starting at hotSpinUs (1200us, NOT the 1150us
  //     ramp floor) and ramping +1us every 100ms until RPM clears the target. The gentle
  //     ramp-from-1200 avoids a sudden impeller deceleration. Held (not increased) once OK.
  bool hot = egt.ok && egt.c > (float)cfg.cooldownTargetC;
  bool purging = (int32_t)(purgeOutUntilMs - millis()) > 0;   // post-fail fuel blow-out window
  bool belowTarget = (!rpmOk || rpmData.rpm < (float)cfg.hotSpinMinRpm);
  if ((hot || purging) && belowTarget) {
    uint32_t now = millis();
    if (hotSpinCurUs < cfg.hotSpinUs) { hotSpinCurUs = cfg.hotSpinUs; hotSpinLastStepMs = now; }  // engage at floor
    else if (now - hotSpinLastStepMs >= 100 && hotSpinCurUs < cfg.startRampToUs) {                 // +1us / 0.1s, capped
      hotSpinLastStepMs = now; hotSpinCurUs += 1;
    }
    if (startUs < hotSpinCurUs) startUs = hotSpinCurUs;
  } else if (hot || purging) {
    // RPM already OK: hold the last hot-spin PWM (don't drop the impeller).
    if (hotSpinCurUs > 0 && startUs < hotSpinCurUs) startUs = hotSpinCurUs;
  } else {
    // Cooled below threshold and not purging: release the ramp so it restarts at 1200us.
    hotSpinCurUs = 0; hotSpinLastStepMs = millis();
  }
}

void checkFailures() {
  if (ecuMode == MODE_WAITING || ecuMode == MODE_COOLDOWN || ecuMode == MODE_ABORTED) return;

  bool fueled = (pumpUs > ESC_SAFE_US + 10) || valve1Cmd || valve2Cmd;

  // Comm watchdog: once the engine is RUNNING (IDLING/OPERATING) at a commanded
  // throttle, require a live operator link (Serial/Web command, or a visible Web UI
  // /api poll) within commTimeoutMs, so a dropped link can't leave it running
  // unattended. MODE_STARTING is deliberately NOT covered: the automated start
  // sequence takes longer than commTimeoutMs and nothing refreshes the link during
  // it, so covering it false-aborts a legitimate Serial-only start; the start is
  // instead bounded by the per-stage timeouts. Button-started runs are exempt.
  if (cfg.commWatchdogEnabled && !runStartedByButton &&
      (ecuMode == MODE_IDLING || ecuMode == MODE_OPERATING) &&
      millis() - lastOperatorLinkMs > cfg.commTimeoutMs) { abortAll("COMM_TIMEOUT"); return; }

  if (!dryStartActive && cfg.abortOnEgtFault && !egt.ok) { abortAll("EGT_FAULT"); return; }
  // OVER_TEMP: also preempt the ~1-sample (EGT_READ_PERIOD_MS) staleness of egt.c
  // with a short look-ahead so a fast rise cannot overshoot maxEgtC by ~one sample.
  if (egt.ok && (egt.c >= (float)cfg.maxEgtC ||
      (egt.gradientCps > 0.0f && (egt.c + EGT_ABORT_LOOKAHEAD_S * egt.gradientCps) >= (float)cfg.maxEgtC))) { abortAll("OVER_TEMP"); return; }
  if (rpmMeasurementUsable() && rpmData.rpm >= cfg.maxRpm) { abortAll("OVERSPEED"); return; }

  // Any fueled state must have a live RPM signal.
  //  - MODE_STARTING (light-off, igniter on): ignition EMI can transiently classify
  //    the signal NOISY; there we require only that pulses are still ARRIVING
  //    (recency within the 1s timeout), not a CLEAN class, so EMI cannot false-abort
  //    a start. A truly dead sensor still trips via recency.
  //  - IDLING/OPERATING (running RPM): use the shorter FUELED_RPM_LOSS_TIMEOUT_MS so
  //    fuel is cut promptly after a real flameout instead of ~1s later.
  static uint32_t rpmNoisySinceMs = 0;   // debounce for NOISY-but-present while running
  // Suppress RPM_SIGNAL_LOST for one loss-timeout after a stats reset (rpmreset /
  // set rpmfilter / set rpmedge / stage transitions): resetRpmStats() zeroes the
  // last-accepted timestamp until the next edge, which would otherwise read as a
  // signal loss and false-abort a running engine.
  if (cfg.abortOnRpmFault && fueled && (millis() - rpmStatsResetAtMs >= FUELED_RPM_LOSS_TIMEOUT_MS)) {
    bool lost = false;
    if (ecuMode == MODE_STARTING) {
      lost = !rpmSignalRecentWithin(RPM_SIGNAL_TIMEOUT_MS);
      rpmNoisySinceMs = 0;
    } else if (ecuMode == MODE_IDLING || ecuMode == MODE_OPERATING) {
      if (!rpmSignalRecentWithin(FUELED_RPM_LOSS_TIMEOUT_MS)) {
        lost = true;                       // real: no accepted pulse within 400ms
        rpmNoisySinceMs = 0;
      } else if (!rpmMeasurementUsable()) {
        // Pulses still arriving but classified NOISY. Debounce: only abort if it
        // persists >= RPM_NOISY_ABORT_MS, so a one-window EMI blip doesn't shut
        // down a healthy running engine.
        if (rpmNoisySinceMs == 0) rpmNoisySinceMs = millis();
        if (millis() - rpmNoisySinceMs >= RPM_NOISY_ABORT_MS) lost = true;
      } else {
        rpmNoisySinceMs = 0;               // signal usable again
      }
    } else {
      rpmNoisySinceMs = 0;
    }
    if (lost) { abortAll("RPM_SIGNAL_LOST"); return; }
  } else {
    rpmNoisySinceMs = 0;
  }

  // Flameout grace is anchored to runningSinceMs (first stable idle of the run), not
  // modeEnteredMs, so repeated IDLING<->OPERATING throttle toggles cannot keep
  // re-arming the grace and masking a real flameout.
  if ((ecuMode == MODE_IDLING || ecuMode == MODE_OPERATING) && fueled && rpmMeasurementUsable() &&
      rpmData.rpm < cfg.flameoutRpm && millis() - runningSinceMs > 1500UL) { abortAll("FLAMEOUT"); return; }
}

void printConfig() {
  Serial.println("===== CONFIG =====");
  Serial.print("autoStartEnabled="); Serial.println(cfg.autoStartEnabled ? "ON" : "OFF");
  Serial.print("pulsesPerRev="); Serial.println(cfg.pulsesPerRev);
  Serial.print("ignitionThresholdC="); Serial.println(cfg.ignitionThresholdC);
  Serial.print("maxEgtC="); Serial.println(cfg.maxEgtC);
  Serial.print("idleRpm="); Serial.println(cfg.idleRpm);
  Serial.print("maxRpm="); Serial.println(cfg.maxRpm);
  Serial.print("rpmTolerance="); Serial.println(cfg.rpmTolerance);
  Serial.print("accelToIdleTimeoutMs="); Serial.println(cfg.accelToIdleTimeoutMs);
  Serial.print("cooldown min/timeout/target/starter="); Serial.print(cfg.cooldownMinMs); Serial.print("ms/"); Serial.print(cfg.cooldownTimeoutMs); Serial.print("ms/"); Serial.print(cfg.cooldownTargetC); Serial.print("C/"); Serial.print(cfg.cooldownStarterUs); Serial.println("us");
  Serial.print("commWatchdogEnabled="); Serial.print(cfg.commWatchdogEnabled ? "ON" : "OFF"); Serial.print(" commTimeoutMs="); Serial.println(cfg.commTimeoutMs);
  Serial.print("fuelTargetRpm="); Serial.println(fuelTargetRpm);
  Serial.print("fuelTargetUs="); Serial.print(fuelTargetUs); Serial.print(" (~"); Serial.print(flowFromUs(fuelTargetUs), 1); Serial.println(" ml/min)");
  Serial.print("accel/decel ms="); Serial.print(cfg.accelStepDelayMs); Serial.print("/"); Serial.print(cfg.decelStepDelayMs);
  Serial.print(" low="); Serial.print(cfg.lowAccelStepDelayMs); Serial.print("/"); Serial.println(cfg.lowDecelStepDelayMs);
  Serial.print("starter purge/spin/assist us="); Serial.print(cfg.starterPurgeUs); Serial.print("/"); Serial.print(cfg.starterSpinUs); Serial.print("/"); Serial.println(cfg.starterAssistUs);
  Serial.print("purge ramp: arm-hold "); Serial.print(cfg.escArmHoldMs); Serial.print("ms at ESC_SAFE, then hold "); Serial.print(cfg.startRampFromUs); Serial.print("us for "); Serial.print(cfg.purgeStableMs); Serial.print("ms, then ramp ->"); Serial.print(cfg.startRampToUs); Serial.print("us +1us/"); Serial.print(cfg.startRampStepMs); Serial.print("ms, arm@"); Serial.print(cfg.ignArmRpm); Serial.println("rpm");
  Serial.print("lightoff: EGT>="); Serial.print(cfg.ignitionThresholdC); Serial.print("C & dEGT>="); Serial.print(cfg.lightOffMinRiseCps); Serial.print("C/s held "); Serial.print(cfg.lightOffConfirmMs); Serial.print("ms; flameProve="); Serial.print(cfg.flameProveMs); Serial.print("ms; flameout drop>"); Serial.print(cfg.flameOutDropC); Serial.println("C/2s");
  Serial.print("assist: "); Serial.print(cfg.starterAssistUs); Serial.print("->"); Serial.print(cfg.startRampToUs); Serial.print("us to "); Serial.print(cfg.starterMaxRpm); Serial.println("rpm then OFF");
  Serial.print("protection: starter OFF above "); Serial.print(cfg.starterMaxRpm); Serial.print(" rpm; while EGT>"); Serial.print(cfg.cooldownTargetC); Serial.print("C hold rpm>="); Serial.print(cfg.hotSpinMinRpm); Serial.print(" via starter@"); Serial.print(cfg.hotSpinUs); Serial.println("us");
  Serial.print("introFuelUs="); Serial.print(cfg.introFuelUs); Serial.print(" (~"); Serial.print(flowFromUs(cfg.introFuelUs), 1); Serial.println(" ml/min)");
  Serial.print("idleFuelUs="); Serial.print(cfg.idleFuelUs); Serial.print(" (~"); Serial.print(flowFromUs(cfg.idleFuelUs), 1); Serial.println(" ml/min)");
  Serial.print("maxFuelUs="); Serial.print(cfg.maxFuelUs); Serial.print(" (~"); Serial.print(flowFromUs(cfg.maxFuelUs), 1); Serial.println(" ml/min)");
  Serial.print("pumpTestUs="); Serial.print(cfg.pumpTestUs); Serial.print(" (~"); Serial.print(flowFromUs(cfg.pumpTestUs), 1); Serial.println(" ml/min, bench pump prime)");
  Serial.print("requireChecklistForStart="); Serial.println(cfg.requireChecklistForStart ? "ON" : "OFF");
  Serial.print("allowDryStartWhenEgtFault="); Serial.println(cfg.allowDryStartWhenEgtFault ? "ON" : "OFF");
  Serial.print("dryStartRunMs="); Serial.println(cfg.dryStartRunMs);
  Serial.print("webEnabled="); Serial.println(cfg.webEnabled ? "ON" : "OFF");
  Serial.print("statusLedGPIO="); Serial.print(PIN_STATUS_LED); Serial.println(STATUS_LED_ACTIVE_LOW ? " active LOW" : " active HIGH");
  Serial.print("sdLoggingEnabled="); Serial.println(cfg.sdLoggingEnabled ? "ON" : "OFF");
  Serial.print("sdOk="); Serial.println(sdOk ? "OK" : "FAIL/NOT_INIT");
  Serial.print("sdLogPath="); Serial.println(sdLogPath);
  Serial.print("sdPins CS/SCK/MOSI/MISO="); Serial.print(PIN_SD_CS); Serial.print("/"); Serial.print(PIN_SD_SCK); Serial.print("/"); Serial.print(PIN_SD_MOSI); Serial.print("/"); Serial.println(PIN_SD_MISO);
  Serial.println("==================");
}

void printHelp() {
  Serial.println("===== COMMANDS =====");
  Serial.println("apijson -> prints the same JSON as web /api, for a PC-side serial<->web bridge (no other side effects)");
  Serial.println("help | status | showcfg | rpmreset | stop | off");
  Serial.println("clearabort            -> acknowledge abort (needs cool EGT)");
  Serial.println("clearabort force      -> acknowledge abort when EGT sensor is dead (dry-start only after)");
  Serial.println("rpmdetail             -> print one detailed RPM diagnostic line");
  Serial.println("rpmdetail on/off      -> show/hide RPM diagnostics in normal status");
  Serial.println("arm2                  -> unlock dangerous commands for 10 sec");
  Serial.println("USER_BTN GPIO22       -> short=status, hold 2s=ARM, hold 3s while armed=START, running press=STOP");
  Serial.println("STATUS_LED GPIO2      -> slow=WAITING, quick=ARMED, solid=START/RUN, fast=ABORT/TEST");
  Serial.println("autostart on/off      -> enable/disable auto-idle runtime");
  Serial.println("ignpulse 500..3000    -> glow ON ms then auto OFF");
  Serial.println("starttest us ms       -> starter test, e.g. starttest 1100 3000");
  Serial.println("pumptest us [ms]      -> bench pump verify only, auto-off after ms (default 1500ms, no cap)");
  Serial.println("startmanual us | startmanual off  -> hold starter PWM (no auto-off, manual page)");
  Serial.println("pumpmanual us | pumpmanual off    -> hold pump PWM + open valve1 (no auto-off, manual page)");
  Serial.println("esccal start | esccal cancel      -> ESC throttle-range calibration: PUMP+STARTER to 2000us, auto-drop to 1000us after 5s");
  Serial.println("esctest start|pump <us>           -> self-test: jumper GPIO25/26 -> GPIO34, measures ESP32's own actual output pulse width via pulseIn() (no scope needed)");
  Serial.println("ign on | ign off                  -> hold glow (no auto-off, manual page)");
  Serial.println("manauto on | manauto off -> manual page AUTO START: requires RPM>=ignArmRpm, then mirrors ST_INTRO_FUEL/LIGHTOFF (valve1+pump+glow -> confirm real light-off -> valve2 -> close valve1 -> prove flame -> glow off)");
  Serial.println("valve1 on/off (Start solenoid, bench-only) | valve2 on/off (Main oil valve, bench-only)");
  Serial.println("startidle             -> guarded auto-idle start sequence");
  Serial.println("set egtstart dry|strict | set drystartms <ms>");
  Serial.println("(manual page glow auto-off reuses lightoffrise/lightoffconfirmms - same real light-off rule as LIGHTOFF)");
  Serial.println("set ppr 1|2 | set intro <us> | set idleus <us> | set maxus <us> | set pumptestus <us>");
  Serial.println("set purgeus <us> | set spinus <us> | set assistus <us> -> starter crank PWM (1000..1500)");
  Serial.println("set rampfromus/ramptous/rampstepms/escarmholdms/purgestablems/ignarmrpm/spinuptimeoutms -> purge ramp");
  Serial.println("set fueldelayms/lightoffrise/lightoffconfirmms/flameprovems/flameoutdropc/accelholdms/accelstepus/purgeoutms -> lightoff/accel");
  Serial.println("set startermaxrpm/hotspinminrpm/hotspinus -> starter protection");
  Serial.println("  (all PWM/limit tuning: intro/idleus/maxus/pumptestus/purgeus/spinus/assistus/idlerpm/maxrpm/rpmtol/maxegt/maxgrad only in WAITING/ABORTED)");
  Serial.println("set idlerpm <rpm> | set maxrpm <rpm> | set rpmtol <rpm> | set maxegt <C> | set maxgrad <C/s>");
  Serial.println("set acceltoidlems <ms> | set cooldownms <minMs> <timeoutMs> | set cooltarget <C> | set coolstarter <us>");
  Serial.println("set throttle <0..100> | set accelms <ms> | set decelms <ms> | set lowaccelms <ms> | set lowdecelms <ms>");
  Serial.println("set commtimeout <3000..60000> -> comm watchdog window (ms) while IDLING/OPERATING");
  Serial.println("set commwatchdog on/off -> abort if no Serial/Web link within commtimeout (arm2 required to disable)");
  Serial.println("set rpmfilter <20..5000>  -> software glitch filter in us");
  Serial.println("set rpmedge rising|falling -> RPM interrupt edge");
  Serial.println("RPM guard: WAITING+outputs OFF + any edge => RPM=0, SIG=REST_NOISE, start blocked");
  Serial.println("set rpmcal on/off -> bench-only: disable/restore the RPM rest-guard to hand-spin the sensor for calibration (blocks arm/start while ON)");
  Serial.println("checklist | resetcheck | confirmkill | set checklist on/off");
  Serial.println("test egt|rpm_noise|ign|starter|starter_ign|valve1|valve2|pump|kill");
  Serial.println("sdstatus | sdtest | set sdlog on/off");
  Serial.println("savecfg | loadcfg      -> save/reload tunable config to /ECUCFG.TXT on SD (auto-loaded on boot)");
  Serial.println("web on | web off");
}

void printRpmDetail() {
  Serial.print("RPM_DETAIL=");
  Serial.print("RPM="); Serial.print(rpmData.rpm, 0);
  Serial.print(" | RPMw="); Serial.print(rpmData.rpmWindow, 0);
  Serial.print(" | RPMp="); Serial.print(rpmData.rpmPeriod, 0);
  Serial.print(" | SIG="); Serial.print(rpmSignalName());
  Serial.print(" | pin="); Serial.print(rpmData.pinLevel);
  Serial.print(" | acc="); Serial.print(rpmData.acceptedWindow);
  Serial.print(" raw="); Serial.print(rpmData.rawEdges);
  Serial.print(" rej="); Serial.print(rpmData.rejectedEdges);
  Serial.print(" rej%="); Serial.print(rpmData.rejectPct, 1);
  Serial.print(" | per="); Serial.print(rpmData.lastPeriodUs); Serial.print("us");
  Serial.print(" | min="); Serial.print(rpmData.minIntervalUs); Serial.print("us");
  Serial.print(" | max="); Serial.print(rpmData.maxIntervalUs); Serial.print("us");
  Serial.print(" | avg="); Serial.print(rpmData.avgIntervalUs, 1); Serial.print("us");
  Serial.print(" | JIT="); Serial.print(rpmData.jitterPct, 1); Serial.print("%");
  Serial.print(" | DIFF="); Serial.print(rpmData.rpmDiffPct, 1); Serial.print("%");
  Serial.print(" | filt="); Serial.print(rpmData.filterUs); Serial.print("us");
  Serial.print(" | edge="); Serial.print(rpmEdgeName());
  Serial.print(" | RNOISE="); Serial.print(rpmNoiseName(rpmData.noise));
  if (rpmData.restPulseNoise) Serial.print(" | REST_ACTIVITY");
  if (rpmData.rejectedEdges > 0) Serial.print(" | NOISE_FAST");
  if (rpmData.jitterPct > 30.0f && rpmData.validIntervals > 5) Serial.print(" | RPM_UNSTABLE");
  Serial.println();
}

void printStatus(bool force = false) {
  if (!force && millis() - lastStatusPrintMs < STATUS_PRINT_MS) return;
  lastStatusPrintMs = millis();
  Serial.print("MODE="); Serial.print(modeName(ecuMode));
  Serial.print(" | STAGE="); Serial.print(stageName(startStage));
  if (dryStartActive) Serial.print(" | DRY=1");
  Serial.print(" | EGT=");
  if (egt.ok) { Serial.print(egt.c, 1); Serial.print("C"); } else { Serial.print("ERR("); Serial.print(egtFaultString(egt.fault)); Serial.print(")"); }
  Serial.print(" | dEGT="); Serial.print(egt.gradientCps, 1);
  Serial.print(" | RPM="); Serial.print(rpmData.rpm, 0);
  Serial.print(" | SIG="); Serial.print(rpmSignalName());
  if (rpmData.noise == RPM_REST_NOISE || rpmData.noise == RPM_NOISY || rpmData.noise == RPM_WARN) {
    Serial.print(" | RNOISE="); Serial.print(rpmNoiseName(rpmData.noise));
  }

  if (rpmDetailMode) {
    Serial.print(" | RPMw="); Serial.print(rpmData.rpmWindow, 0);
    Serial.print(" | RPMp="); Serial.print(rpmData.rpmPeriod, 0);
    Serial.print(" | pin="); Serial.print(rpmData.pinLevel);
    Serial.print(" | acc="); Serial.print(rpmData.acceptedWindow);
    Serial.print(" raw="); Serial.print(rpmData.rawEdges);
    Serial.print(" rej="); Serial.print(rpmData.rejectedEdges);
    Serial.print(" rej%="); Serial.print(rpmData.rejectPct, 1);
    Serial.print(" | per="); Serial.print(rpmData.lastPeriodUs); Serial.print("us");
    Serial.print(" | JIT="); Serial.print(rpmData.jitterPct, 1); Serial.print("%");
    Serial.print(" | DIFF="); Serial.print(rpmData.rpmDiffPct, 1); Serial.print("%");
    Serial.print(" | filt="); Serial.print(rpmData.filterUs); Serial.print("us");
    Serial.print(" | edge="); Serial.print(rpmEdgeName());
    Serial.print(" | RNOISE="); Serial.print(rpmNoiseName(rpmData.noise));
  }

  Serial.print(" | RTGT="); Serial.print(fuelTargetRpm);
  Serial.print(" | PUMP="); Serial.print(pumpUs); Serial.print("us ~"); Serial.print(flowFromUs(pumpUs), 1); Serial.print("ml/min");
  Serial.print(" | FTGT="); Serial.print(fuelTargetUs); Serial.print("us ~"); Serial.print(flowFromUs(fuelTargetUs), 1); Serial.print("ml/min");
  Serial.print(" | START="); Serial.print(startUs); Serial.print("us");
  Serial.print(" | IGN="); Serial.print(ignCmd ? 1 : 0);
  Serial.print(" | V1="); Serial.print(valve1Cmd ? 1 : 0);
  Serial.print(" | V2="); Serial.print(valve2Cmd ? 1 : 0);
  Serial.print(" | SD="); Serial.print(sdOk ? (cfg.sdLoggingEnabled ? "OK" : "OFF") : "FAIL");
  Serial.print(" | THR="); Serial.print(throttlePct); Serial.print("%");
  Serial.print(" | AUTO="); Serial.print(cfg.autoStartEnabled ? "ON" : "OFF");
  Serial.print(" | ARM="); Serial.print(isStage2Armed() ? "ON" : "OFF");
  if (cfg.requireChecklistForStart) { String ck; Serial.print(" | CHECK="); Serial.print(checklistPassed(ck) ? "PASS" : "BLOCK"); }
  Serial.print(" | ABORT="); Serial.print(lastAbortReason);

  if (rpmDetailMode) {
    if (rpmData.restPulseNoise) Serial.print(" | REST_ACTIVITY");
    if (rpmData.rejectedEdges > 0) Serial.print(" | NOISE_FAST");
    if (rpmData.jitterPct > 30.0f && rpmData.validIntervals > 5) Serial.print(" | RPM_UNSTABLE");
  }

  Serial.println();
}

long numberAfter(const String& cmd, const String& prefix) { return cmd.substring(prefix.length()).toInt(); }
bool parseTwoInts(const String& cmd, int& a, uint32_t& b) {
  int p1 = cmd.indexOf(' '), p2 = cmd.indexOf(' ', p1 + 1); if (p1 < 0 || p2 < 0) return false;
  a = cmd.substring(p1 + 1, p2).toInt(); b = (uint32_t)cmd.substring(p2 + 1).toInt(); return true;
}

void handleCommand(String cmd) {
  lastOperatorLinkMs = millis();  // any Serial/Web command counts as proof the operator is present
  runStartedByButton = false;     // a remote operator is now driving -> comm watchdog applies again
  cmd.trim(); cmd.toLowerCase(); if (!cmd.length()) return;
  if (cmd == "help") { printHelp(); return; }
  if (cmd == "status") { printStatus(true); return; }
  if (cmd == "rpmdetail") { updateRpm(); printRpmDetail(); return; }
  if (cmd == "rpmdetail on") { rpmDetailMode = true; resetRpmStats(); Serial.println("RPM detail mode ON."); return; }
  if (cmd == "rpmdetail off") { rpmDetailMode = false; Serial.println("RPM detail mode OFF."); return; }
  if (cmd == "showcfg" || cmd == "cfg") { printConfig(); return; }
  if (cmd == "sdstatus") { printSdStatus(); return; }
  if (cmd == "sdtest") { addLog("SDTEST manual event"); Serial.println("SDTEST event written if SD=OK."); return; }
  if (cmd == "savecfg") { saveConfigToSd(); return; }
  if (cmd == "loadcfg") { if (loadConfigFromSd()) { resetRpmStats(); attachRpmInterrupt(); Serial.println("Config reloaded from SD."); } else Serial.println("No saved config applied."); return; }
  if (cmd == "set sdlog on") { cfg.sdLoggingEnabled = true; addLog("SD LOG ON"); Serial.println("SD logging ON"); return; }
  if (cmd == "set sdlog off") { addLog("SD LOG OFF"); cfg.sdLoggingEnabled = false; Serial.println("SD logging OFF"); return; }
  if (cmd == "rpmreset") { resetRpmStats(); return; }
  if (cmd == "checklist") { printChecklist(); return; }
  if (cmd == "resetcheck") { resetChecklist(); return; }
  if (cmd == "confirmkill") { setChecklist(TEST_KILL, TEST_PASS, "Physical NC kill switch confirmed by user"); return; }
  if (cmd.startsWith("test ")) { runTestByName(cmd.substring(5)); return; }
  if (cmd == "web on") { startWebServer(); return; }
  if (cmd == "web off") { stopWebServer(); return; }
  if (cmd == "stop") { requestStop(); return; }
  if (cmd == "off" || cmd == "stage2off") { stage2Off(); return; }
  if (cmd == "clearabort" || cmd == "clearabort force") {
    if (ecuMode != MODE_ABORTED && ecuMode != MODE_WAITING) { Serial.println("ERROR: clearabort only valid in ABORTED/WAITING mode."); return; }
    bool force = (cmd == "clearabort force");
    // Plain clearabort needs a valid, cool thermocouple. "clearabort force" also
    // works when the sensor is dead (egt.ok=false) so a broken thermocouple can
    // never trap the ECU in ABORTED; it still refuses when EGT is readable and hot.
    bool allowed = force ? egtAllowsDeliberateAbortClear() : egtSafeForAbortClear();
    if (!allowed) {
      Serial.print("CLEARABORT BLOCKED: ");
      if (egt.ok) { Serial.print("EGT still hot ("); Serial.print(egt.c, 1); Serial.println("C). Wait for engine to cool."); }
      else { Serial.print("EGT sensor fault ("); Serial.print(egtFaultString(egt.fault)); Serial.println("). Verify engine is cool, then use: clearabort force"); }
      return;
    }
    abortAcknowledged = true;
    if (force && !egt.ok) Serial.println("WARNING: EGT sensor faulty - temperature NOT verified. Next start will be DRY only (fuel/ignition OFF).");
    Serial.print("ABORT acknowledged (was: "); Serial.print(lastAbortReason); Serial.println("). ECU ready to arm.");
    return;
  }
  if (cmd == "arm2") { armStage2(); return; }
  if (cmd == "autostart on") { if (!isStage2Armed()) { Serial.println("ERROR: type arm2 first."); return; } cfg.autoStartEnabled = true; Serial.println("AUTO-START ENABLED."); return; }
  if (cmd == "autostart off") { cfg.autoStartEnabled = false; Serial.println("AUTO-START DISABLED."); return; }
  if (cmd == "startidle") { if (!isStage2Armed()) { Serial.println("ERROR: type arm2 first."); return; } String why; if (!canStartAutoIdle(why)) { Serial.print("START BLOCKED: "); Serial.println(why); return; } if (why != "OK") { Serial.print("START NOTICE: "); Serial.println(why); } beginAutoIdle(); return; }

  if (cmd.startsWith("ignpulse ")) {
    if (ecuMode != MODE_WAITING && ecuMode != MODE_ABORTED) { Serial.println("ERROR: ignpulse only while WAITING/ABORTED."); return; }
    // Do not energize the igniter into a still-hot engine (e.g. ABORTED after a
    // cooldown timeout that expired while hot): residual fuel + a hot core is a
    // re-light hazard. Starter-only tests below stay allowed (they aid cooling).
    if (fuelCommandBlockedByHotEgt()) {
      Serial.print("IGNPULSE BLOCKED: engine still hot (EGT="); Serial.print(egt.c, 1);
      Serial.print("C > cooldown target "); Serial.print(cfg.cooldownTargetC); Serial.println("C).");
      return;
    }
    uint32_t ms = numberAfter(cmd, "ignpulse "); if (ms < 500 || ms > 3000) { Serial.println("ERROR: ignpulse 500..3000 ms"); return; }
    ignCmd = true; manualIgnOffAtMs = millis() + ms; applyOutputs(); Serial.print("GLOW ON for "); Serial.print(ms); Serial.println(" ms"); return;
  }

  if (cmd.startsWith("starttest ")) {
    if (ecuMode != MODE_WAITING && ecuMode != MODE_ABORTED) { Serial.println("ERROR: starttest only while WAITING/ABORTED."); return; }
    int us; uint32_t ms; if (!parseTwoInts(cmd, us, ms)) { Serial.println("ERROR: use starttest <us> <ms>"); return; }
    // Duration is not capped: bench manual test runs exactly the ms the user enters.
    // (us still bounded for ESC safety.) Stop early anytime with 'off'/'stop'.
    if (us < 1000 || us > 1300) { Serial.println("ERROR: us 1000..1300"); return; }
    startUs = us; manualStartOffAtMs = millis() + ms; applyOutputs(); Serial.print("STARTER TEST RUNNING for "); Serial.print(ms); Serial.println(" ms"); return;
  }

  if (cmd.startsWith("pumptest ")) {
    if (ecuMode != MODE_WAITING && ecuMode != MODE_ABORTED) { Serial.println("ERROR: pumptest only while WAITING/ABORTED."); return; }

    String args = cmd.substring(String("pumptest ").length());
    args.trim();
    int sp = args.indexOf(' ');
    int us = (sp < 0) ? args.toInt() : args.substring(0, sp).toInt();
    uint32_t ms = (sp < 0) ? 1500UL : (uint32_t)args.substring(sp + 1).toInt();

    // Duration is not capped: bench manual pump test runs exactly the ms the user
    // enters. (us still bounded for ESC safety.) Stop early anytime with 'off'/'stop'.
    if (us < 1000 || us > 1225) { Serial.println("ERROR: bench pumptest limited 1000..1225 us"); return; }
    if (fuelCommandBlockedByHotEgt()) {
      Serial.print("PUMPTEST BLOCKED: engine still hot (EGT="); Serial.print(egt.c, 1);
      Serial.print("C > cooldown target "); Serial.print(cfg.cooldownTargetC);
      Serial.println("C). Let it cool before spraying fuel.");
      return;
    }

    pumpUs = us;
    fuelTargetUs = us;
    fuelValvesAuto(true);
    manualPumpOffAtMs = millis() + ms;
    applyOutputs();
    Serial.print("PUMP TEST RUNNING ");
    Serial.print(us);
    Serial.print("us ~");
    Serial.print(flowFromUs(us), 1);
    Serial.print(" ml/min for ");
    Serial.print(ms);
    Serial.println(" ms, then AUTO OFF");
    return;
  }

  // ---- Manual page: continuous holds (no duration, no auto-off) for starter/pump/glow,
  // paralleling how valve1/valve2 on/off already work. Stop with the matching "off".
  if (cmd.startsWith("startmanual ")) {
    if (ecuMode != MODE_WAITING && ecuMode != MODE_ABORTED) { Serial.println("ERROR: startmanual only while WAITING/ABORTED."); return; }
    String arg = cmd.substring(String("startmanual ").length()); arg.trim();
    if (arg == "off") { startUs = ESC_SAFE_US; manualStartOffAtMs = 0; applyOutputs(); Serial.println("STARTER MANUAL OFF."); return; }
    int us = arg.toInt();
    if (us < ESC_MIN_US || us > ESC_MAX_US) { Serial.print("ERROR: startmanual us "); Serial.print(ESC_MIN_US); Serial.print(".."); Serial.print(ESC_MAX_US); Serial.println(" (or 'startmanual off')"); return; }
    startUs = us; manualStartOffAtMs = 0; applyOutputs();
    Serial.print("STARTER MANUAL HOLD at "); Serial.print(us); Serial.println("us (no auto-off - 'startmanual off' to stop)");
    return;
  }

  if (cmd.startsWith("pumpmanual ")) {
    if (ecuMode != MODE_WAITING && ecuMode != MODE_ABORTED) { Serial.println("ERROR: pumpmanual only while WAITING/ABORTED."); return; }
    String arg = cmd.substring(String("pumpmanual ").length()); arg.trim();
    if (arg == "off") { pumpUs = ESC_SAFE_US; fuelTargetUs = ESC_SAFE_US; fuelValvesAuto(false); manualPumpOffAtMs = 0; applyOutputs(); Serial.println("PUMP MANUAL OFF."); return; }
    int us = arg.toInt();
    if (us < ESC_MIN_US || us > ESC_MAX_US) { Serial.print("ERROR: pumpmanual us "); Serial.print(ESC_MIN_US); Serial.print(".."); Serial.print(ESC_MAX_US); Serial.println(" (or 'pumpmanual off')"); return; }
    if (fuelCommandBlockedByHotEgt()) {
      Serial.print("PUMPMANUAL BLOCKED: engine still hot (EGT="); Serial.print(egt.c, 1);
      Serial.print("C > cooldown target "); Serial.print(cfg.cooldownTargetC); Serial.println("C).");
      return;
    }
    pumpUs = us; fuelTargetUs = us; fuelValvesAuto(true); manualPumpOffAtMs = 0; applyOutputs();
    Serial.print("PUMP MANUAL HOLD at "); Serial.print(us); Serial.print("us ~"); Serial.print(flowFromUs(us), 1);
    Serial.println(" ml/min (no auto-off - 'pumpmanual off' to stop)");
    return;
  }

  // ESC self-test: measures the ESP32's OWN actual output pulse width with
  // pulseIn() on a loopback wire, for when a scope can't reliably read a 50Hz
  // signal (DSO152 case). Operator must jumper a wire from PIN_ESC_START/PIN_ESC_PUMP
  // to PIN_ESC_SELFTEST_IN (GPIO34) - in parallel with the existing ESC feed, does
  // not disturb it. pulseIn() blocks briefly (up to ~150ms for 5 samples), only
  // ever run manually at rest in WAITING/ABORTED.
  if (cmd.startsWith("esctest ")) {
    if (ecuMode != MODE_WAITING && ecuMode != MODE_ABORTED) { Serial.println("ERROR: esctest only while WAITING/ABORTED."); return; }
    String rest = cmd.substring(String("esctest ").length()); rest.trim();
    int sp = rest.indexOf(' ');
    if (sp < 0) { Serial.println("ERROR: use esctest start|pump <us>"); return; }
    String which = rest.substring(0, sp);
    int us = rest.substring(sp + 1).toInt();
    if (us < ESC_MIN_US || us > ESC_MAX_US) { Serial.print("ERROR: esctest us "); Serial.print(ESC_MIN_US); Serial.print(".."); Serial.println(ESC_MAX_US); return; }
    if (which != "start" && which != "pump") { Serial.println("ERROR: use esctest start|pump <us>"); return; }
    if (which == "start") { startUs = us; } else { pumpUs = us; }
    applyOutputs();
    Serial.print("ESCTEST: jumper GPIO"); Serial.print(which == "start" ? PIN_ESC_START : PIN_ESC_PUMP);
    Serial.print(" -> GPIO"); Serial.print(PIN_ESC_SELFTEST_IN);
    Serial.println(" now if not already done. Measuring actual pulse width (5 samples)...");
    pinMode(PIN_ESC_SELFTEST_IN, INPUT);
    uint32_t minUs = 0xFFFFFFFFUL, maxUs = 0, sumUs = 0; int okCount = 0;
    for (int i = 0; i < 5; i++) {
      uint32_t w = pulseIn(PIN_ESC_SELFTEST_IN, HIGH, 30000UL); // 30ms timeout covers the 20ms nominal period
      if (w > 0) { okCount++; sumUs += w; if (w < minUs) minUs = w; if (w > maxUs) maxUs = w; }
    }
    if (okCount == 0) {
      Serial.println("ESCTEST: NO PULSE seen on loopback pin - check jumper wire/GPIO34, or the output truly isn't toggling.");
    } else {
      Serial.print("ESCTEST: commanded="); Serial.print(us); Serial.print("us | measured min=");
      Serial.print(minUs); Serial.print("us avg="); Serial.print(sumUs / (uint32_t)okCount);
      Serial.print("us max="); Serial.print(maxUs); Serial.print("us over "); Serial.print(okCount); Serial.println("/5 samples");
    }
    if (which == "start") { startUs = ESC_SAFE_US; } else { pumpUs = ESC_SAFE_US; }
    applyOutputs();
    return;
  }

  // ESC throttle-range calibration: drives both PUMP and STARTER ESCs to
  // ESC_MAX_US so the operator can (re)power them and have them learn the
  // endpoint, then this ECU auto-drops to ESC_MIN_US after ESC_CAL_MAX_HOLD_MS
  // - no separate receiver/bench rig needed. Valves stay closed (no
  // fuelValvesAuto call) since this only calibrates PWM range, not fuel flow.
  if (cmd == "esccal start") {
    if (ecuMode != MODE_WAITING && ecuMode != MODE_ABORTED) { Serial.println("ERROR: esccal only while WAITING/ABORTED."); return; }
    escCalActive = true;
    manualStartOffAtMs = 0; manualPumpOffAtMs = 0;
    startUs = ESC_MAX_US; pumpUs = ESC_MAX_US;
    applyOutputs();
    escCalPhaseUntilMs = millis() + ESC_CAL_MAX_HOLD_MS;
    addLog("ESC CAL: MAX 2000us (PUMP+STARTER)");
    Serial.print("ESC CAL: phat MAX 2000us tren PUMP+STARTER. Cap/mo nguon ESC ngay (nghe tit tit). Tu dong ha ve MIN sau ");
    Serial.print(ESC_CAL_MAX_HOLD_MS / 1000); Serial.println("s.");
    return;
  }
  if (cmd == "esccal cancel" || cmd == "esccal off") {
    escCalActive = false; escCalPhaseUntilMs = 0;
    startUs = ESC_SAFE_US; pumpUs = ESC_SAFE_US; applyOutputs();
    Serial.println("ESC CAL: da huy, ve an toan (1000us).");
    return;
  }

  if (cmd == "ign on") {
    if (ecuMode != MODE_WAITING && ecuMode != MODE_ABORTED) { Serial.println("ERROR: ign only while WAITING/ABORTED."); return; }
    if (fuelCommandBlockedByHotEgt()) {
      Serial.print("IGN BLOCKED: engine still hot (EGT="); Serial.print(egt.c, 1); Serial.println("C).");
      return;
    }
    ignCmd = true; manualIgnOffAtMs = 0; applyOutputs(); Serial.println("GLOW ON (no auto-off - 'ign off' to stop).");
    return;
  }
  if (cmd == "ign off") { ignCmd = false; manualIgnOffAtMs = 0; applyOutputs(); Serial.println("GLOW OFF."); return; }

  if (cmd == "manauto on") {
    if (ecuMode != MODE_WAITING && ecuMode != MODE_ABORTED) { Serial.println("ERROR: manauto only while WAITING/ABORTED."); return; }
    if (fuelCommandBlockedByHotEgt()) {
      Serial.print("MANUAL AUTO START BLOCKED: engine still hot (EGT="); Serial.print(egt.c, 1); Serial.println("C).");
      return;
    }
    // Manual page has no starter/PURGE stage of its own - require the engine already
    // spinning above ignArmRpm (same RPM the real PURGE stage requires) before letting
    // fuel/glow in, same reason: don't introduce fuel into a stalled/barely-turning engine.
    if (!rpmMeasurementUsable() || rpmData.rpm < (float)cfg.ignArmRpm) {
      Serial.print("MANUAL AUTO START BLOCKED: RPM "); Serial.print(rpmMeasurementUsable() ? rpmData.rpm : 0.0f, 0);
      Serial.print(" < ignArmRpm "); Serial.print(cfg.ignArmRpm); Serial.println(" - spin the engine up first (starter manual).");
      return;
    }
    valve1Cmd = true; valve2Cmd = false;
    pumpUs = cfg.introFuelUs; fuelTargetUs = cfg.introFuelUs;
    ignCmd = true;
    manualIgnOffAtMs = manualStartOffAtMs = manualPumpOffAtMs = 0;
    manualGlowRiseSinceMs = 0; // manAuto owns the glow-off decision now, not the standalone safety net
    manAutoPhase = 0; manAutoPhaseMs = millis(); manAutoRiseSinceMs = 0; manAutoStartedMs = millis();
    flameRefMs = 0;
    manAutoActive = true;
    applyOutputs();
    Serial.println("MANUAL AUTO START: Valve1 open, pump at introFuelUs, glow ON - waiting for real light-off...");
    return;
  }
  if (cmd == "manauto off") {
    manAutoActive = false; ignCmd = false; pumpUs = ESC_SAFE_US; fuelTargetUs = ESC_SAFE_US; fuelValvesAuto(false);
    applyOutputs();
    Serial.println("MANUAL AUTO START cancelled.");
    return;
  }

  if (cmd == "valve1 on") { if (fuelCommandBlockedByHotEgt()) { Serial.print("VALVE1 BLOCKED: engine still hot (EGT="); Serial.print(egt.c, 1); Serial.println("C)."); return; } valve1Cmd = true; applyOutputs(); Serial.println("VALVE1 ON (no auto-off - turn off manually)"); return; }
  if (cmd == "valve1 off") { valve1Cmd = false; applyOutputs(); Serial.println("VALVE1 OFF"); return; }
  if (cmd == "valve2 on") { if (fuelCommandBlockedByHotEgt()) { Serial.print("VALVE2 BLOCKED: engine still hot (EGT="); Serial.print(egt.c, 1); Serial.println("C)."); return; } valve2Cmd = true; applyOutputs(); Serial.println("VALVE2 ON (no auto-off - turn off manually)"); return; }
  if (cmd == "valve2 off") { valve2Cmd = false; applyOutputs(); Serial.println("VALVE2 OFF"); return; }

  if (cmd.startsWith("set rpmfilter ")) {
    int f = numberAfter(cmd, "set rpmfilter ");
    if (f < 20 || f > 5000) { Serial.println("ERROR: rpmfilter 20..5000 us"); return; }
    noInterrupts(); rpmMinPulseUs = (uint32_t)f; interrupts();
    resetRpmStats();
    Serial.print("rpmFilterUs="); Serial.println(f);
    return;
  }

  if (cmd.startsWith("set rpmedge ")) {
    String e = cmd.substring(String("set rpmedge ").length());
    e.trim();
    if (e == "rising") rpmEdgeMode = RISING;
    else if (e == "falling") rpmEdgeMode = FALLING;
    else { Serial.println("ERROR: use set rpmedge rising|falling"); return; }
    resetRpmStats();
    attachRpmInterrupt();
    Serial.print("rpmEdge="); Serial.println(rpmEdgeName());
    return;
  }

  if (cmd == "set rpmcal on") {
    if (ecuMode != MODE_WAITING && ecuMode != MODE_ABORTED) { Serial.println("ERROR: set rpmcal only while WAITING/ABORTED."); return; }
    rpmCalMode = true; resetRpmStats();
    Serial.println("RPM CAL MODE ON: rest-guard disabled, RPM readout now live even while WAITING (bench sensor calibration only - 'set rpmcal off' when done, before arming).");
    addLog("RPM CAL MODE ON");
    return;
  }
  if (cmd == "set rpmcal off") { rpmCalMode = false; resetRpmStats(); Serial.println("RPM CAL MODE OFF: rest-guard restored."); addLog("RPM CAL MODE OFF"); return; }
  if (cmd == "set checklist on") { cfg.requireChecklistForStart = true; Serial.println("Checklist interlock ON"); addLog("CHECKLIST INTERLOCK ON"); return; }
  if (cmd == "set checklist off") { if (!isStage2Armed()) { Serial.println("ERROR: type arm2 first."); return; } cfg.requireChecklistForStart = false; Serial.println("Checklist interlock OFF - DEBUG ONLY"); addLog("CHECKLIST INTERLOCK OFF"); return; }
  if (cmd == "set egtstart dry") { cfg.allowDryStartWhenEgtFault = true; Serial.println("EGT start mode = DRY: EGT fault converts startidle to dry starter/RPM test, no fuel/valves/ign."); addLog("EGT START MODE DRY"); return; }
  if (cmd == "set egtstart strict") { cfg.allowDryStartWhenEgtFault = false; Serial.println("EGT start mode = STRICT: EGT fault blocks startidle."); addLog("EGT START MODE STRICT"); return; }
  if (cmd.startsWith("set drystartms ")) { int v = numberAfter(cmd, "set drystartms "); if (v < 1000 || v > 15000) { Serial.println("ERROR: drystartms 1000..15000"); return; } cfg.dryStartRunMs = (uint32_t)v; Serial.println("OK"); return; }
  if (cmd.startsWith("set ppr ")) { if (ecuMode != MODE_WAITING && ecuMode != MODE_ABORTED) { Serial.println("ERROR: set ppr only in WAITING/ABORTED (rescales live RPM)."); return; } int p = numberAfter(cmd, "set ppr "); if (p != 1 && p != 2) { Serial.println("ERROR: ppr 1 or 2"); return; } cfg.pulsesPerRev = p; resetRpmStats(); Serial.println("OK"); return; }
  // Bench-only tuning guard: block PWM / RPM / EGT-limit setters while the engine
  // is running, so a stray Serial/Web command cannot move a live setpoint or the
  // protection envelope. Tune in WAITING/ABORTED only.
  if ((cmd.startsWith("set intro ") || cmd.startsWith("set idleus ") || cmd.startsWith("set maxus ") ||
       cmd.startsWith("set purgeus ") || cmd.startsWith("set spinus ") || cmd.startsWith("set assistus ") ||
       cmd.startsWith("set idlerpm ") || cmd.startsWith("set maxrpm ") || cmd.startsWith("set rpmtol ") ||
       cmd.startsWith("set maxegt ") || cmd.startsWith("set maxgrad ") || cmd.startsWith("set pumptestus ") ||
       cmd.startsWith("set rampfromus ") || cmd.startsWith("set ramptous ") || cmd.startsWith("set rampstepms ") || cmd.startsWith("set escarmholdms ") || cmd.startsWith("set purgestablems ") ||
       cmd.startsWith("set ignarmrpm ") || cmd.startsWith("set spinuptimeoutms ") || cmd.startsWith("set fueldelayms ") ||
       cmd.startsWith("set lightoffrise ") || cmd.startsWith("set lightoffconfirmms ") || cmd.startsWith("set flameprovems ") ||
       cmd.startsWith("set flameoutdropc ") || cmd.startsWith("set accelholdms ") || cmd.startsWith("set accelstepus ") ||
       cmd.startsWith("set purgeoutms ") ||
       cmd.startsWith("set startermaxrpm ") || cmd.startsWith("set hotspinminrpm ") || cmd.startsWith("set hotspinus ")) &&
      ecuMode != MODE_WAITING && ecuMode != MODE_ABORTED) {
    Serial.println("ERROR: PWM/limit tuning only in WAITING/ABORTED (not while running).");
    return;
  }
  if (cmd.startsWith("set intro ")) { int us = numberAfter(cmd, "set intro "); if (us < 1000 || us > 1250) { Serial.println("ERROR: intro 1000..1250"); return; } cfg.introFuelUs = us; Serial.println("OK"); return; }
  if (cmd.startsWith("set idleus ")) { int us = numberAfter(cmd, "set idleus "); if (us < 1000 || us > 1270) { Serial.println("ERROR: idleus 1000..1270"); return; } cfg.idleFuelUs = us; if (cfg.maxFuelUs < us) cfg.maxFuelUs = us; Serial.println("OK"); return; }
  if (cmd.startsWith("set maxus ")) { int us = numberAfter(cmd, "set maxus "); if (us < 1100 || us > 1300) { Serial.println("ERROR: maxus 1100..1300"); return; } cfg.maxFuelUs = max(us, cfg.idleFuelUs); Serial.println("OK"); return; }
  if (cmd.startsWith("set pumptestus ")) { int us = numberAfter(cmd, "set pumptestus "); if (us < 1000 || us > 1225) { Serial.println("ERROR: pumptestus 1000..1225"); return; } cfg.pumpTestUs = us; Serial.println("OK"); return; }
  if (cmd.startsWith("set purgeus ")) { int us = numberAfter(cmd, "set purgeus "); if (us < 1000 || us > 1500) { Serial.println("ERROR: purgeus 1000..1500"); return; } cfg.starterPurgeUs = us; Serial.println("OK"); return; }
  if (cmd.startsWith("set spinus ")) { int us = numberAfter(cmd, "set spinus "); if (us < 1000 || us > 1500) { Serial.println("ERROR: spinus 1000..1500"); return; } cfg.starterSpinUs = us; Serial.println("OK"); return; }
  if (cmd.startsWith("set assistus ")) { int us = numberAfter(cmd, "set assistus "); if (us < 1000 || us > 1500) { Serial.println("ERROR: assistus 1000..1500"); return; } cfg.starterAssistUs = us; Serial.println("OK"); return; }
  if (cmd.startsWith("set rampfromus ")) { int us = numberAfter(cmd, "set rampfromus "); if (us < 1000 || us > 1400) { Serial.println("ERROR: rampfromus 1000..1400"); return; } cfg.startRampFromUs = us; if (cfg.startRampToUs < us) cfg.startRampToUs = us; Serial.println("OK"); return; }
  if (cmd.startsWith("set ramptous ")) { int us = numberAfter(cmd, "set ramptous "); if (us < 1000 || us > 1500) { Serial.println("ERROR: ramptous 1000..1500"); return; } cfg.startRampToUs = max(us, cfg.startRampFromUs); Serial.println("OK"); return; }
  if (cmd.startsWith("set rampstepms ")) { int v = numberAfter(cmd, "set rampstepms "); if (v < 10 || v > 5000) { Serial.println("ERROR: rampstepms 10..5000"); return; } cfg.startRampStepMs = (uint32_t)v; Serial.println("OK"); return; }
  if (cmd.startsWith("set escarmholdms ")) { int v = numberAfter(cmd, "set escarmholdms "); if (v < 0 || v > 10000) { Serial.println("ERROR: escarmholdms 0..10000"); return; } cfg.escArmHoldMs = (uint32_t)v; Serial.println("OK"); return; }
  if (cmd.startsWith("set purgestablems ")) { int v = numberAfter(cmd, "set purgestablems "); if (v < 0 || v > 60000) { Serial.println("ERROR: purgestablems 0..60000"); return; } cfg.purgeStableMs = (uint32_t)v; Serial.println("OK"); return; }
  if (cmd.startsWith("set ignarmrpm ")) { int r = numberAfter(cmd, "set ignarmrpm "); if (r < 500 || r > 60000) { Serial.println("ERROR: ignarmrpm 500..60000"); return; } cfg.ignArmRpm = r; Serial.println("OK"); return; }
  if (cmd.startsWith("set spinuptimeoutms ")) { int v = numberAfter(cmd, "set spinuptimeoutms "); if (v < 5000 || v > 180000) { Serial.println("ERROR: spinuptimeoutms 5000..180000"); return; } cfg.spinupRpmTimeoutMs = (uint32_t)v; Serial.println("OK"); return; }
  if (cmd.startsWith("set fueldelayms ")) { int v = numberAfter(cmd, "set fueldelayms "); if (v < 0 || v > 20000) { Serial.println("ERROR: fueldelayms 0..20000"); return; } cfg.fuelDelayMs = (uint32_t)v; Serial.println("OK"); return; }
  if (cmd.startsWith("set lightoffrise ")) { int v = numberAfter(cmd, "set lightoffrise "); if (v < 0 || v > 500) { Serial.println("ERROR: lightoffrise 0..500"); return; } cfg.lightOffMinRiseCps = v; Serial.println("OK"); return; }
  if (cmd.startsWith("set lightoffconfirmms ")) { int v = numberAfter(cmd, "set lightoffconfirmms "); if (v < 0 || v > 10000) { Serial.println("ERROR: lightoffconfirmms 0..10000"); return; } cfg.lightOffConfirmMs = (uint32_t)v; Serial.println("OK"); return; }
  if (cmd.startsWith("set flameprovems ")) { int v = numberAfter(cmd, "set flameprovems "); if (v < 0 || v > 20000) { Serial.println("ERROR: flameprovems 0..20000"); return; } cfg.flameProveMs = (uint32_t)v; Serial.println("OK"); return; }
  if (cmd.startsWith("set flameoutdropc ")) { int v = numberAfter(cmd, "set flameoutdropc "); if (v < 1 || v > 500) { Serial.println("ERROR: flameoutdropc 1..500"); return; } cfg.flameOutDropC = v; Serial.println("OK"); return; }
  if (cmd.startsWith("set accelholdms ")) { int v = numberAfter(cmd, "set accelholdms "); if (v < 200 || v > 60000) { Serial.println("ERROR: accelholdms 200..60000"); return; } cfg.accelHoldMs = (uint32_t)v; Serial.println("OK"); return; }
  if (cmd.startsWith("set accelstepus ")) { int v = numberAfter(cmd, "set accelstepus "); if (v < 1 || v > 50) { Serial.println("ERROR: accelstepus 1..50"); return; } cfg.accelStepUs = v; Serial.println("OK"); return; }
  if (cmd.startsWith("set purgeoutms ")) { int v = numberAfter(cmd, "set purgeoutms "); if (v < 0 || v > 60000) { Serial.println("ERROR: purgeoutms 0..60000"); return; } cfg.purgeOutMs = (uint32_t)v; Serial.println("OK"); return; }
  if (cmd.startsWith("set startermaxrpm ")) { int r = numberAfter(cmd, "set startermaxrpm "); if (r < 3000 || r > 60000) { Serial.println("ERROR: startermaxrpm 3000..60000"); return; } cfg.starterMaxRpm = r; Serial.println("OK"); return; }
  if (cmd.startsWith("set hotspinminrpm ")) { int r = numberAfter(cmd, "set hotspinminrpm "); if (r < 0 || r > 20000) { Serial.println("ERROR: hotspinminrpm 0..20000"); return; } cfg.hotSpinMinRpm = r; Serial.println("OK"); return; }
  if (cmd.startsWith("set hotspinus ")) { int us = numberAfter(cmd, "set hotspinus "); if (us < 1000 || us > 1400) { Serial.println("ERROR: hotspinus 1000..1400"); return; } cfg.hotSpinUs = us; Serial.println("OK"); return; }
  if (cmd.startsWith("set idlerpm ")) { int r = numberAfter(cmd, "set idlerpm "); if (r < 10000 || r > 60000) { Serial.println("ERROR: idlerpm 10000..60000"); return; } if (r > cfg.maxRpm - 5000) { Serial.println("ERROR: idlerpm must be <= maxRpm-5000"); return; } cfg.idleRpm = r; Serial.println("OK"); return; }
  if (cmd.startsWith("set maxrpm ")) { int r = numberAfter(cmd, "set maxrpm "); if (r < cfg.idleRpm + 5000 || r > 160000) { Serial.println("ERROR: maxrpm must be idleRpm+5000..160000"); return; } cfg.maxRpm = r; Serial.println("OK"); return; }
  if (cmd.startsWith("set rpmtol ")) { int r = numberAfter(cmd, "set rpmtol "); if (r < 500 || r > 15000) { Serial.println("ERROR: rpmtol 500..15000"); return; } cfg.rpmTolerance = r; Serial.println("OK"); return; }
  if (cmd.startsWith("set maxegt ")) { int t = numberAfter(cmd, "set maxegt "); if (t < 400 || t > 950) { Serial.println("ERROR: maxegt 400..950"); return; } cfg.maxEgtC = t; Serial.println("OK"); return; }
  if (cmd.startsWith("set maxgrad ")) { int g = numberAfter(cmd, "set maxgrad "); if (g < 50 || g > 1000) { Serial.println("ERROR: maxgrad 50..1000 C/s"); return; } cfg.maxTempGradientCps = g; Serial.println("OK"); return; }
  if (cmd.startsWith("set acceltoidlems ")) { int v = numberAfter(cmd, "set acceltoidlems "); if (v < 5000 || v > 60000) { Serial.println("ERROR: acceltoidlems 5000..60000"); return; } cfg.accelToIdleTimeoutMs = (uint32_t)v; Serial.println("OK"); return; }
  if (cmd.startsWith("set cooltarget ")) { int v = numberAfter(cmd, "set cooltarget "); if (v < 50 || v > 250) { Serial.println("ERROR: cooltarget 50..250 C"); return; } cfg.cooldownTargetC = v; Serial.println("OK"); return; }
  if (cmd.startsWith("set coolstarter ")) { int v = numberAfter(cmd, "set coolstarter "); if (v < 1000 || v > 1200) { Serial.println("ERROR: coolstarter 1000..1200 us"); return; } cfg.cooldownStarterUs = v; Serial.println("OK"); return; }
  if (cmd.startsWith("set cooldownms ")) {
    // NOTE: this command has a 3-word prefix, so the generic parseTwoInts() (which
    // splits on the first two spaces) would misparse "cooldownms" as the first int.
    // Parse the two values from after the prefix explicitly.
    String a = cmd.substring(String("set cooldownms ").length()); a.trim();
    int sp = a.indexOf(' ');
    if (sp < 0) { Serial.println("ERROR: use set cooldownms <minMs> <timeoutMs>"); return; }
    int minMs = a.substring(0, sp).toInt();
    uint32_t timeoutMs = (uint32_t)a.substring(sp + 1).toInt();
    if (minMs < 1000 || minMs > 30000 || timeoutMs < 5000 || timeoutMs > 120000 || timeoutMs < (uint32_t)minMs) { Serial.println("ERROR: minMs 1000..30000, timeoutMs 5000..120000 and >= minMs"); return; }
    cfg.cooldownMinMs = (uint32_t)minMs; cfg.cooldownTimeoutMs = timeoutMs; Serial.println("OK"); return;
  }
  if (cmd.startsWith("set accelms ")) { int v = numberAfter(cmd, "set accelms "); if (v < 50 || v > 2000) { Serial.println("ERROR: accelms 50..2000"); return; } cfg.accelStepDelayMs = v; Serial.println("OK"); return; }
  if (cmd.startsWith("set decelms ")) { int v = numberAfter(cmd, "set decelms "); if (v < 50 || v > 2000) { Serial.println("ERROR: decelms 50..2000"); return; } cfg.decelStepDelayMs = v; Serial.println("OK"); return; }
  if (cmd.startsWith("set lowaccelms ")) { int v = numberAfter(cmd, "set lowaccelms "); if (v < 100 || v > 3000) { Serial.println("ERROR: lowaccelms 100..3000"); return; } cfg.lowAccelStepDelayMs = v; Serial.println("OK"); return; }
  if (cmd.startsWith("set lowdecelms ")) { int v = numberAfter(cmd, "set lowdecelms "); if (v < 100 || v > 3000) { Serial.println("ERROR: lowdecelms 100..3000"); return; } cfg.lowDecelStepDelayMs = v; Serial.println("OK"); return; }
  if (cmd.startsWith("set throttle ")) { throttlePct = constrain((int)numberAfter(cmd, "set throttle "), 0, 100); Serial.println("OK"); return; }
  if (cmd.startsWith("set commtimeout ")) { int v = numberAfter(cmd, "set commtimeout "); if (v < 3000 || v > 60000) { Serial.println("ERROR: commtimeout 3000..60000 ms"); return; } cfg.commTimeoutMs = (uint32_t)v; Serial.println("OK"); return; }
  if (cmd == "set commwatchdog on") { cfg.commWatchdogEnabled = true; Serial.println("Comm watchdog ON"); addLog("COMM WATCHDOG ON"); return; }
  if (cmd == "set commwatchdog off") { if (!isStage2Armed()) { Serial.println("ERROR: type arm2 first."); return; } cfg.commWatchdogEnabled = false; Serial.println("Comm watchdog OFF - DEBUG ONLY"); addLog("COMM WATCHDOG OFF"); return; }

  Serial.println("Unknown command. Type help");
}

void setup() {
  Serial.begin(115200);
  delay(400);
  pinMode(PIN_IGN, OUTPUT); pinMode(PIN_VALVE_1, OUTPUT); pinMode(PIN_VALVE_2, OUTPUT);
  pinMode(PIN_STATUS_LED, OUTPUT); writeStatusLed(false);
  pinMode(PIN_USER_BTN, INPUT_PULLUP); // external 10k pull-up is also OK; button shorts to GND
  pinMode(PIN_RPM, INPUT);
  // Load saved tuning from SD before wiring the RPM interrupt, so a persisted
  // rpmedge/rpmfilter takes effect on this boot. Values are clamped on load.
  loadConfigFromSd();
  attachRpmInterrupt();
  escPumpPeriodUs = escAttach(PIN_ESC_PUMP, LEDC_CH_PUMP);
  escStartPeriodUs = escAttach(PIN_ESC_START, LEDC_CH_START);
  Serial.print("ESC PWM actual period: pump="); Serial.print(escPumpPeriodUs, 2);
  Serial.print("us start="); Serial.print(escStartPeriodUs, 2); Serial.println("us (nominal 20000us)");
  forceSafeOutputs();
  lastOperatorLinkMs = millis();
  bool thermoOk = thermo.begin();
  thermo.setFaultChecks(MAX31855_FAULT_ALL);
  enterWaitingSafe();
  initSdLogging();
  addLog("BOOT Test ECU V1 WebUI/TestWizard");
  Serial.println("Test ECU V1 WebUI/TestWizard booted.");
  if (cfg.webEnabled) startWebServer();
  Serial.print("MAX31855 begin() = "); Serial.println(thermoOk ? "OK" : "CHECK_WIRING");
  Serial.print("RPM edge = "); Serial.println(rpmEdgeName());
  Serial.print("RPM filter = "); Serial.print((uint32_t)rpmMinPulseUs); Serial.println(" us");
  Serial.println("RPM detail mode = OFF. Use rpmdetail or rpmdetail on.");
  Serial.println("USER_BTN GPIO22: short=status, hold 2s=ARM, hold 3s while armed=START, running press=SOFT STOP, aborted hold 2s=CLEAR.");
  Serial.println("STATUS_LED GPIO2: active-low, slow=WAITING, quick=ARMED, solid=START/RUN, fast=ABORT/TEST.");
  Serial.print("SD_LOG: "); Serial.print(sdOk ? "OK file=" : "FAIL file="); Serial.println(sdLogPath);
  Serial.println("EGT OPEN behavior: startidle => DRY START/RPM TEST only, no fuel/valves/ign. Use 'set egtstart strict' to block instead.");
  Serial.println("Outputs safe. Auto-start disabled. Type help.");
  delay(2500); // give ESCs safe 1000us pulse
}

void loop() {
  if (webStarted) server.handleClient();
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (serialCmdBuf.length()) {
        // Read-only status poll for the PC-side serial<->web bridge: bypasses
        // handleCommand() entirely (same as the real HTTP /api route bypassing it)
        // so a passive status poll cannot reset the button-start comm-watchdog
        // exemption or count as "operator just issued a command" the way a real
        // command does.
        if (serialCmdBuf == "apijson") Serial.println(webStatusJson());
        else handleCommand(serialCmdBuf);
        serialCmdBuf = "";
      }
    } else if (c != '\r') {
      if (serialCmdBuf.length() < 80) serialCmdBuf += c;
    }
  }
  handleUserButton();
  if (manualIgnOffAtMs > 0 && millis() >= manualIgnOffAtMs) { ignCmd = false; manualIgnOffAtMs = 0; applyOutputs(); Serial.println("GLOW AUTO OFF."); }
  if (manualStartOffAtMs > 0 && millis() >= manualStartOffAtMs) { startUs = ESC_SAFE_US; manualStartOffAtMs = 0; applyOutputs(); Serial.println("STARTER AUTO OFF."); }
  if (manualPumpOffAtMs > 0 && millis() >= manualPumpOffAtMs) { pumpUs = ESC_SAFE_US; fuelTargetUs = ESC_SAFE_US; fuelValvesAuto(false); manualPumpOffAtMs = 0; applyOutputs(); Serial.println("PUMP AUTO OFF."); }
  if (escCalActive && escCalPhaseUntilMs > 0 && millis() >= escCalPhaseUntilMs) {
    startUs = ESC_MIN_US; pumpUs = ESC_MIN_US; applyOutputs();
    escCalPhaseUntilMs = 0; escCalActive = false;
    addLog("ESC CAL: MIN 1000us (done)");
    Serial.println("ESC CAL: da ha ve MIN 1000us. Cho nghe nhac chao ngan tu ESC de xac nhan hoan tat.");
  }
  updateActiveTest();
  isStage2Armed();
  updateRpm(); updateEgt();
  // Manual page safety: glow held ON by hand (bench test) - apply the exact same
  // "real light-off" confirmation used by ST_LIGHTOFF phase 0 (EGT>=ignitionThresholdC
  // AND rising >= lightOffMinRiseCps, held continuously for lightOffConfirmMs), so a
  // real flame is what turns the glow off here too, not just glow radiant heat.
  if (ignCmd && !manAutoActive && (ecuMode == MODE_WAITING || ecuMode == MODE_ABORTED)) {
    bool rising = egt.ok && egt.c >= (float)cfg.ignitionThresholdC &&
                  egt.gradientCps >= (float)cfg.lightOffMinRiseCps;
    if (rising) {
      if (manualGlowRiseSinceMs == 0) manualGlowRiseSinceMs = millis();
      else if (millis() - manualGlowRiseSinceMs >= cfg.lightOffConfirmMs) {
        ignCmd = false; manualIgnOffAtMs = 0; manualGlowRiseSinceMs = 0; applyOutputs();
        Serial.print("GLOW AUTO OFF: real light-off detected (EGT="); Serial.print(egt.c, 0);
        Serial.print("C, dEGT="); Serial.print(egt.gradientCps, 0); Serial.println("C/s).");
      }
    } else {
      manualGlowRiseSinceMs = 0;
    }
  } else {
    manualGlowRiseSinceMs = 0;
  }
  updateManAuto();
  switch (ecuMode) {
    case MODE_WAITING: break;
    case MODE_STARTING: updateStarting(); break;
    case MODE_IDLING: updateIdling(); break;
    case MODE_OPERATING: updateOperating(); break;
    case MODE_COOLDOWN: updateCooldown(); break;
    case MODE_ABORTED: break;
    default: abortAll("INVALID_MODE"); break;
  }
  checkFailures();
  applyStarterProtection();   // hot soak-back floor + starter overspeed cutoff (all modes)
  applyOutputs();
  updateStatusLed();
  updateSdTelemetry();
  flushSdEventQueue();   // blocking SD writes happen here, after the safety checks
  printStatus(false);
}
