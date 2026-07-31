# ECU Manual V1 Firmware — Code Architecture & Understanding Guide

**Firmware**: `ECU_ManualV1.ino` (Version 4.2)
**Tools**: `index.html` (Web Serial Dashboard)
**Platform**: ESP32 (NodeMCU-32S), Arduino core 2.x **or** 3.x
**Language**: Arduino C++

> **Note on line numbers**: this guide navigates by **function / symbol name**, not
> line numbers — the firmware evolves and hard line references drift. Use your
> editor's symbol search. A companion changelog of behaviour changes lives in
> `CODE_REVIEW_FINDINGS.md`.

---

## 🏗️ Overall Architecture

State-machine firmware. One non-blocking `loop()` drives everything:

```
loop():
  1. server.handleClient()          (Web UI, if started)
  2. drain Serial -> handleCommand()
  3. handleUserButton()
  4. manual auto-off timers          (ign / starter / pump / valve1 / valve2)
  5. updateActiveTest()              (Test Wizard countdowns)
  6. isStage2Armed()                 (auto-disarm after 10s)
  7. updateRpm(); updateEgt()        (sensors)
  8. mode switch:  updateStarting / updateIdling / updateOperating / updateCooldown
  9. checkFailures()                 (all aborts evaluated here)
  10. applyOutputs()                 (write ESC PWM + digital outputs)
  11. updateStatusLed()
  12. updateSdTelemetry(); flushSdEventQueue()   (deferred SD writes)
  13. printStatus(false)             (throttled serial status)
```

Fuel/RPM/EGT control is re-evaluated every loop; safety checks (`checkFailures`)
run **after** the mode update and **before** `applyOutputs()`.

---

## ⚙️ PWM Driver — raw LEDC (not ESP32Servo)

ESC outputs (pump + starter) are driven by the ESP32 **LEDC** peripheral directly,
**not** the `ESP32Servo` library (which produced an unstable pulse on some core
versions and stuttered the ESC — see `CLAUDE.md`).

```cpp
escAttach(pin, legacyChannel)   // ledcAttach() on core >=3, ledcSetup+ledcAttachPin on core <3
escWriteUs(pin, legacyChannel, us)  // duty = us * 65535 / 20000  (50 Hz, 16-bit)
```
- 50 Hz frame, 16-bit resolution → 1000 µs = duty 3276, 2000 µs = duty 6553.
- Core version detected via `ESP_ARDUINO_VERSION_MAJOR`.

> **Arduino gotcha**: because `escAttach`/`escWriteUs` are the first functions in the
> sketch, the IDE injects auto-generated prototypes ahead of the enum definitions.
> The state enums (`EcuMode`, `StartStage`, `RpmNoiseLevel`, `TestId`, `TestResult`)
> are therefore **forward-declared** near the top so those prototypes compile.

---

## 🎯 Core State Machine

### ECU Modes
```cpp
enum EcuMode : uint8_t {
  MODE_WAITING, MODE_STARTING, MODE_IDLING,
  MODE_OPERATING, MODE_COOLDOWN, MODE_ABORTED
};
```

### Start Stages
```cpp
enum StartStage : uint8_t {
  ST_NONE, ST_PURGE, ST_SPINUP_PREHEAT,
  ST_INTRO_FUEL, ST_POST_IGNITION_HEAT, ST_ACCEL_TO_IDLE
};
```

### Mode transitions
```
WAITING
  │ arm (button hold 2s / arm2) then start (button hold 3s / startidle)
  ▼
STARTING  (ST_PURGE → ST_SPINUP_PREHEAT → ST_INTRO_FUEL → ST_POST_IGNITION_HEAT → ST_ACCEL_TO_IDLE)
  │ idle RPM reached  (enterMode records runningSinceMs here)
  ▼
IDLING ⇄ OPERATING   (throttle >0 / =0 toggles; does NOT reset runningSinceMs)
  │ soft stop / any abort
  ▼
COOLDOWN (starter airflow, fuel/ign/valves OFF)
  │ min time + (EGT cool OR timeout)
  ▼
WAITING            (after soft stop)   or   ABORTED (after an abort)
```

`ABORTED` is left only by `clearabort` (sets `abortAcknowledged`) or the physical
button hold. `stop`/`off` do **not** clear ABORTED (interlock).

---

## 📊 Key Data Structures

### Config (`struct Config cfg`)
Runtime configuration; the struct values are **power-on defaults**, most are
overridable at runtime via `set ...` (Serial/Web). Notable fields:

| Field | Default | Meaning |
|-------|---------|---------|
| `autoStartEnabled` | false | Auto-idle at boot (off) |
| `requireEgtForStart` / `allowDryStartWhenEgtFault` | true | EGT gating / dry-start fallback |
| `requireRpmForStart`, `abortOnEgtFault`, `abortOnRpmFault` | true | Abort enables |
| `ignitionThresholdC` | 100 | "combusting" / ignition threshold |
| `maxEgtC` | 680 | OVER_TEMP abort limit |
| `maxTempGradientCps` | 200 | EGT rise-rate limit (steady state only) — `set maxgrad` |
| `idleRpm` / `maxRpm` / `rpmTolerance` | 42000 / 110000 / 5000 | RPM targets & deadband |
| `flameoutRpm` / `starterReleaseRpm` | 15000 / 24000 | flame-out / starter release |
| `starterPurgeUs` / `starterSpinUs` / `starterAssistUs` | 1100 / 1200 / 1200 | crank PWM — `set purgeus/spinus/assistus` |
| `introFuelUs` | 1160 (~50 ml/min) | light-off fuel dose (kept **< idle**) — `set intro` |
| `idleFuelUs` / `maxFuelUs` | 1175 / 1260 | idle / max fuel — `set idleus/maxus` |
| `pumpTestUs` | 1210 | bench pump-prime PWM, **separate** from introFuelUs — `set pumptestus` |
| `commWatchdogEnabled` / `commTimeoutMs` | true / 8000 | operator-link watchdog |
| cooldown / accel-to-idle / ramp timings | — | various `set ...` |

> All PWM/limit tuning setters are **rejected unless WAITING/ABORTED** so a stray
> command can't move a live setpoint or the protection envelope mid-run.

### EgtState `egt`
`ok, c, prevC, gradientCps, fault, lastReadMs, lastGoodMs`.

### RpmData `rpmData`
`rpm` (chosen source), `rpmPeriod` (instantaneous), `rpmWindow` (100 ms average),
`avgIntervalUs`, `jitterPct` (CV = stddev/mean), `rejectPct`, `rpmDiffPct`,
`rawEdges/acceptedWindow/rejectedEdges`, `noise` (RpmNoiseLevel), `signalRecent`,
`restPulseNoise`.

---

## 🔌 Pin Configuration

| Pin | Function | Dir | Type |
|-----|----------|-----|------|
| 18 / 5 / 19 | EGT CLK / CS / DO | — | SPI (MAX31855) |
| 33 | RPM sensor | IN | Digital (ISR) |
| 26 | Pump ESC | OUT | LEDC PWM 50 Hz |
| 25 | Starter ESC | OUT | LEDC PWM 50 Hz |
| 17 | Valve 1 (Start solenoid) | OUT | Digital |
| 16 | Valve 2 (Main oil) | OUT | Digital |
| 32 | Ignition / glow | OUT | Digital |
| 22 | User button | IN | Digital (active-low) |
| 2 | Status LED | OUT | Digital (active-low) |
| 13 / 14 / 23 / 27 | SD CS / SCK / MOSI / MISO | — | SPI |

Valve roles (per EnJet E86/G3 manual): **Valve1** = start solenoid, open only
during `MODE_STARTING`; **Valve2** = main oil valve, open whenever fuel is commanded.

---

## 🔋 Subsystems

### 1. EGT (MAX31855)
- `updateEgt()` polls every `EGT_READ_PERIOD_MS` (120 ms); `readError()` then
  `readCelsius()`; `isnan`/fault → `egt.ok=false`, gradient zeroed.
- `gradientCps` = ΔT/Δt.
- **Two look-aheads with different jobs**:
  - Fuel control: 3 s projected temp (`egt.c + 3·gradient ≥ maxEgtC`) — steady state.
  - OVER_TEMP abort: short `EGT_ABORT_LOOKAHEAD_S = 0.2 s` — compensates ~1 sample of
    read staleness so a fast rise can't overshoot `maxEgtC` by a sample.
- **Stuck-sensor hint** (log only, never aborts): if `egt.c` is byte-for-byte
  unchanged for `EGT_STUCK_WARN_MS` (6 s) while combusting, logs a warning — a
  frozen-but-valid thermocouple isn't caught by open/short fault detection.
- **Dry-start**: if EGT is OPEN, `startidle` becomes a starter/RPM-only test
  (pump/valves/ign forced OFF every loop); real fuel start still needs valid EGT.

### 2. RPM sensing (ISR on GPIO 33)
- `rpmISR()` timestamps each edge with `micros()` (coerced to non-zero to avoid the
  "no edge yet" sentinel colliding at the ~71 min wrap).
- **Two-stage de-glitch**: (a) fixed min quiet-period `rpmMinPulseUs` (120 µs default,
  `set rpmfilter`); (b) adaptive half-period mask — reject an edge closer than half
  the last accepted period (an isolated EMI spike after a quiet gap).
- `updateRpm()` snapshots ISR counters inside `noInterrupts()` (the only `uint64_t`
  accumulators are read atomically there), computes:
  - `rpmWindow = accepted · 60e6 / (windowUs · ppr)`  (100 ms average)
  - `rpmPeriod = 60e6 / (lastPeriod · ppr)`  (instantaneous, preferred when recent)
  - `jitterPct` = **coefficient of variation** (stddev/mean), so one stray interval
    doesn't swamp an otherwise-clean signal.
- **REST guard**: while WAITING with all outputs safe, isolated edges are treated as
  noise (`RPM_REST_NOISE`, control RPM forced 0) to stop 1–3 false pulses reading as RPM.

Noise classes (`classifyRpmNoise`):
```
NO_SIGNAL : no recent signal and raw==0
NOISY     : raw>1 & accepted==0 (a lone post-reset priming edge is NOT noise), OR rejected>=3/rejectPct>20, OR jitter>30% (>=5 intervals), OR rpmDiff>30%
WARN      : rejected>0/rejectPct>5,  OR jitter>15% (>=3 intervals),           OR rpmDiff>15%
CLEAN     : otherwise
REST_NOISE: rest-guard active
```

### 3. Fuel control (hybrid, closed-loop to RPM)
```
target RPM ──► fuelTargetUs (nudged ±fuelStepUs vs RPM error, gated by EGT)
                    │
             pumpUs ── ramps toward target 1 µs / step (accel/decel delays)
```
- Pump map (`kPumpMap`, µs → ml/min), interpolated by `flowFromUs`/`usFromFlow`:
  `1000→0, 1160→50, 1175→80, 1250→265, 1260→280, 1265→360, 1270→560, 1300→600`.
- **Over-temp emergency cut**: when `egtRequestsFuelCut()` fires, `pumpUs` steps down
  by `fuelCutStepUs` at the fast decel rate (~40 µs/s), not the old ~2 µs/s ramp that
  could never keep up before the hard abort.
- **Governed ceiling**: `updateOperating()` targets `maxRpm − rpmTolerance` at 100%
  throttle, so full throttle doesn't drive the engine into its own OVERSPEED abort.
- During `MODE_STARTING` the gradient / 3 s look-ahead fuel limiting is **skipped**
  (a fast EGT rise at light-off is expected); only absolute `maxEgtC` + the abort
  look-ahead protect there, so the start can actually reach idle.

### 4. User button (GPIO 22, active-low, 35 ms debounce)
```
WAITING : short=status · hold 2s=ARM
ARMED   : hold 3s=START IDLE           (button runs set runStartedByButton)
STARTING/IDLING/OPERATING : any press=SOFT STOP
ABORTED : hold 2s=clear (if EGT safe/deliberate)
```

### 5. Output control
`applyOutputs()` constrains ESC values to 1000–2000 µs and writes PWM + digital
outputs. `forceSafeOutputs()` slams everything safe (ESC 1000 µs, ign/valves OFF,
clears the valve auto-off timers; the ign/starter/pump timers are cleared in
`stage2Off()`/`beginAutoIdle()`). Manual bench outputs (ignpulse/starttest/
pumptest/valve-on) all have loop auto-off timers.

---

## 🚀 Start Sequence

```
ST_PURGE (purgeTimeMs 3s)         starter=starterPurgeUs(1100), no fuel/ign
ST_SPINUP_PREHEAT (preheatMs)     starter=starterSpinUs(1200), igniter ON (preheat); needs cranking RPM (starterProveMinRpm)
ST_INTRO_FUEL                     igniter still ON, valves open, fuel=introFuelUs(1160); wait ignition/fuel-confirm
ST_POST_IGNITION_HEAT             hold fuel, watch RPM rise; starter -> starterAssistUs(1200)
ST_ACCEL_TO_IDLE                  closed-loop fuel to idleRpm; starter releases; timeout=accelToIdleTimeoutMs(20s)
  └─ idle stable ► MODE_IDLING
```
Every stage is time-bounded (no infinite hang). Dry-start runs the same stages with
fuel/valves/ign forced OFF and its own RPM-test timeouts.

---

## 🔒 Safety — `checkFailures()` (runs every loop, not in WAITING/COOLDOWN/ABORTED)

Order and conditions:
1. **COMM_TIMEOUT** — only while RUNNING (IDLING/OPERATING) with no operator link
   within `commTimeoutMs`. **Exempt** for button-started runs (`runStartedByButton`);
   re-engages on any remote command. MODE_STARTING is deliberately NOT covered (the
   automated start outlasts `commTimeoutMs` with nothing to refresh the link, so
   covering it would false-abort a Serial-only start); the start is bounded instead
   by the per-stage timeouts.
2. **EGT_FAULT** — sensor invalid (unless dry-start).
3. **OVER_TEMP** — `egt.c ≥ maxEgtC` OR the 0.2 s look-ahead projects past it.
4. **OVERSPEED** — usable RPM `≥ maxRpm`.
5. **RPM_SIGNAL_LOST** — while fueled: in STARTING, pulse-recency only (so ignition
   EMI classifying NOISY doesn't false-abort a light-off); in IDLING/OPERATING, no
   pulse within `FUELED_RPM_LOSS_TIMEOUT_MS` (400 ms → fast flameout cut), OR a
   NOISY-but-present signal sustained ≥ `RPM_NOISY_ABORT_MS` (300 ms debounce, so a
   one-window EMI blip can't shut down a healthy engine). Suppressed for one
   loss-timeout after any `resetRpmStats()` so a stats reset can't read as loss.
   The same recency-not-NOISY logic is used by the SPINUP starter-prove checks.
6. **FLAMEOUT** — RPM `< flameoutRpm` after a 1.5 s grace **anchored to
   `runningSinceMs`** (first stable idle), so throttle toggles can't re-arm/mask it.

Other interlocks:
- `requestStop()` acts only in STARTING/IDLING/OPERATING (no ABORTED backdoor).
- `stage2Off()`/`off` in ABORTED, **and** in COOLDOWN while `cooldownAfterAbort` is
  true, only re-asserts safe outputs and does not transition to WAITING — it lets
  a post-abort cooldown finish into ABORTED instead of shortcutting past it;
  requires `clearabort` either way.
- `ignpulse` and `valve1/valve2 on` are blocked while EGT is hot
  (`fuelCommandBlockedByHotEgt`); `starttest` (starter only) stays allowed. The
  Test Wizard's `test ign`/`test starter_ign`/`test valve1`/`test valve2` apply the
  same hot-EGT guard (`test starter` is exempt, same reasoning).
- All outputs OFF on boot; ESC held at 1000 µs during arm.

---

## 🌐 Web UI (SoftAP)

- SSID `ECU_TestV1`, password `admin1234` (**weak default — change `WEB_PASS`
  before live use**; anyone on the AP can arm/start via `/cmd`), IP `192.168.4.1`.
- Routes: `/` (dashboard), `/api` (JSON status, `?act=0` when tab hidden so a
  walked-away session times out instead of keeping the watchdog alive), `/cmd?c=...`
  (runs `handleCommand`, same interlocks as Serial).
- Dashboard: status cards, controls, **Test Wizard**, **Tune quick set** and
  **Starter & Fuel PWM** panels whose inputs **auto-populate from live config**
  (so clicking Set can't silently change a limit), **Starter Manual Test**, event log.
- `webStatusJson()` builds the payload by hand (no ArduinoJson); `jsonEscape()`
  escapes `"` `\` and all control chars.

---

## 💾 SD Card Logging

- `sdLogEvent()` snapshots a CSV line and queues it (ring buffer, drop-oldest); the
  **blocking** `SD.open/println/close` happens in `flushSdEventQueue()` and
  `updateSdTelemetry()` late in the loop, after safety checks.
- **Backoff**: after 5 consecutive write failures `sdOk` is cleared so a pulled card
  stops blocking the control loop every cycle (re-enable requires reboot).
- Telemetry ~2 lines/s while active; CSV columns include time, mode, stage, EGT,
  gradient, RPM, fuel target, pump/start µs, ign, valves, throttle, abort reason.

---

## ✅ Test Wizard / Checklist

Nine tests (`test egt|rpm_noise|ign|starter|starter_ign|valve1|valve2|pump|kill`),
results `TEST_NOT_RUN/RUNNING/PASS/FAIL`. Optionally required before `startidle`
(`set checklist on/off`). Timed tests auto-off; `beginAutoIdle()` cancels any active
test so a countdown can't fire mid-start.

---

## 🎮 Serial / Web Commands (selected)

```
help | status | showcfg | rpmreset | stop | off
arm2 | autostart on/off | startidle | clearabort [force]
rpmdetail [on|off]
ignpulse <ms> | starttest <us> <ms> | pumptest <us> [ms]      (arm2, WAITING/ABORTED; EGT-hot blocked where fuel/ign)
valve1 on/off | valve2 on/off                                 (auto-off 10s; EGT-hot blocked)
test <name> | checklist | resetcheck | confirmkill | set checklist on/off
set intro/idleus/maxus/pumptestus <us>                        (WAITING/ABORTED only)
set purgeus/spinus/assistus <us>   (1000..1500)               (WAITING/ABORTED only)
set idlerpm/maxrpm/rpmtol <rpm> | set maxegt <C> | set maxgrad <C/s>   (WAITING/ABORTED only)
set acceltoidlems/cooldownms/cooltarget/coolstarter ...
set accelms/decelms/lowaccelms/lowdecelms | set throttle <0..100>
set commtimeout <ms> | set commwatchdog on/off
set rpmfilter <us> | set rpmedge rising|falling | set ppr 1|2  (ppr: WAITING/ABORTED only, rescales live RPM)
set egtstart dry|strict | set drystartms <ms>
```

---

## 🔌 Status LED (GPIO 2, active-low)

| Pattern | Meaning |
|---------|---------|
| Very fast | Component test running |
| Fast | Abort/error |
| Quick (~200 ms toggle) | Armed |
| Solid | Starting / running |
| Medium | Idle stable |
| Faster | Cooldown |
| Slow heartbeat | Waiting |

---

## 🐛 Debugging Tips

- **RPM quality**: `rpmdetail on` → want rejectPct <5%, jitter <15%, NOISE=CLEAN.
  A value pinned regardless of PWM ⇒ input frequency isn't changing (EMI at the
  fixed 50 Hz PWM, or a current-limited motor) — see the DSO152 section of
  `COMMISSIONING_GUIDE.md`.
- **EGT**: NaN/fault → check thermocouple + SPI; `egtFaultString()` decodes faults.
- **Fuel flow**: `pumptest`, compare SD `pump_us` against `kPumpMap`.
- **Starter torque**: `starttest`; tune `set purgeus/spinus/assistus`.

---

**Document Version**: 1.1
**Last Updated**: 2026-07-18
**Firmware**: ECU_TestV1_EGT_DRY_START_PATCH (raw-LEDC PWM, runtime-tunable, no kick)
