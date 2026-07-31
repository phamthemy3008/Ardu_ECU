# Kết Quả Code Review - ECU_TestV1_EGT_DRY_START_PATCH.ino

**Ngày review**: 2026-07-16 (review gốc; xem các mục cập nhật cuối file cho thay đổi mới nhất)  
**File**: `ECU_TestV1_EGT_DRY_START_PATCH.ino` (~2290 dòng)  
**Trạng thái**: Sẵn sàng upload ESP32 — với lưu ý các vấn đề bên dưới

---

## 🔴 HIGH — Lỗi Logic An Toàn Nghiêm Trọng

### 1. `stage2Off()` không kiểm tra nhiệt độ EGT — Dòng 485
**Vấn đề**: Lệnh `off` qua Serial và nút "SAFE OFF" trên Web UI đều gọi `stage2Off()` → `enterWaitingSafe()` mà **không kiểm tra EGT còn nóng không**.  
**Nguy hiểm**: Sau khi bị ABORT do OVER_TEMP (EGT 650°C), operator có thể nhấn "SAFE OFF" → vào WAITING → re-arm ngay lập tức trong khi động cơ vẫn rất nóng.  
**Đường an toàn hiện tại**: Button vật lý kiểm tra `egtSafeForAbortClear()` (dòng 589) — nhưng Web UI thì không.  
**Khuyến nghị**: Thêm kiểm tra EGT vào `stage2Off()` trước khi gọi `enterWaitingSafe()`.

```cpp
// FIX gợi ý tại dòng ~485:
void stage2Off() {
  if (!egtSafeForAbortClear()) {
    Serial.println("STAGE2 OFF blocked: EGT still hot");
    return; // Không cho phép tắt khi EGT còn nóng
  }
  enterWaitingSafe();
}
```

---

### 2. Restart từ ABORTED không yêu cầu xác nhận lỗi — Dòng 1395
**Vấn đề**: `canStartAutoIdle()` cho phép start khi `ecuMode == MODE_ABORTED`. Chỉ kiểm tra EGT an toàn (dòng 1402), không yêu cầu operator xác nhận nguyên nhân abort (OVERSPEED, RPM_SIGNAL_LOST, v.v.).  
**Nguy hiểm**: Operator có thể gõ `startidle` ngay sau khi bị OVERSPEED abort — restart ngay mà không hiểu tại sao máy dừng.  
**Khuyến nghị**: Yêu cầu `clearAbort` command riêng trước `startidle`, hoặc tối thiểu log cảnh báo rõ ràng.

---

## 🟡 MEDIUM — Vấn Đề Ảnh Hưởng Đến Độ Tin Cậy

### 3. Pulse đầu tiên sau `resetRpmStats()` bỏ qua bộ lọc glitch — Dòng 740–745
**Vấn đề**: Trong ISR, khi `isrLastRawEdgeUs == 0` (sau `resetRpmStats()`), pulse đầu tiên được chấp nhận vô điều kiện, bất kể độ rộng.  
**Ảnh hưởng**: EMI spike từ igniter ở đầu giai đoạn `ST_SPINUP_PREHEAT` (wet start) có thể được đếm là RPM hợp lệ, che khuất thực tế starter không quay đủ.  
**Khuyến nghị**: Gọi `resetRpmStats()` trước cả wet start spin-up (tương tự dry start ở dòng 1457), không chỉ dry start.

### 4. `egt.gradientCps` bị stale khi EGT sensor lỗi — Dòng 882–886
**Vấn đề**: Khi `egt.ok = false`, biến `gradientCps` không được reset về 0. Các hàm control đọc `egt.ok` trước (an toàn), nhưng Web UI `/api` và `printStatus()` hiển thị gradient cũ mà không đánh dấu là invalid.  
**Ảnh hưởng**: Operator thấy gradient đang hiển thị "30 °C/s" nhưng thực ra EGT sensor đã bị lỗi từ lâu — gây nhầm lẫn khi debug.  
**Khuyến nghị**: Reset `egt.gradientCps = 0` khi `egt.ok = false` trong `updateEgt()`.

### 5. `Serial.readStringUntil('\n')` có thể block loop tới 1 giây — Dòng 1897
**Vấn đề**: Hàm `readStringUntil` chờ đến 1000ms nếu không có newline (gõ chậm, kết nối bị ngắt quãng).  
**Ảnh hưởng**: Trong 1 giây đó, `updateEgt()`, `updateRpm()`, `checkFailures()`, và `applyOutputs()` đều không chạy — **mất phản hồi an toàn trong 1 giây**.  
**Khuyến nghị**: Thay bằng non-blocking serial command buffer, đọc từng ký tự với `Serial.read()` và chỉ xử lý khi có `\n`.

---

## 🟢 LOW — Cải Tiến Nhỏ

### 6. SD file slot bị đầy → ghi đè file cũ — Dòng 995–998
Nếu đã có đủ 1000 files (ECU000.CSV đến ECU999.CSV), vòng lặp tìm tên file kết thúc với `i=999` vẫn trỏ vào file có sẵn. Header mới sẽ được append vào cuối file đó, làm hỏng log cũ.

### 7. Double SPI read → fault code có thể sai — Dòng 880–881
`thermo.readCelsius()` và `thermo.readError()` là 2 transaction SPI riêng biệt. Nếu lỗi thoáng qua giữa 2 lần đọc, `egt.fault = 0` trong khi `egt.ok = false` → hiển thị "SPI/WIRING" thay vì tên lỗi thực.

### 8. Jitter metric bị phóng đại bởi một outlier — Dòng 846–847
`jitterPct = (maxDt - minDt) / avgInterval * 100`. Một khoảng dài bất thường duy nhất trong cửa sổ 100ms làm jitter tăng cao sai, có thể báo `RPM_WARN` / `RPM_NOISY` khi tín hiệu thực ra sạch — có thể block start qua `rpmNoiseBlocksStart()`.

---

## ✅ Điểm Tốt — Giữ Nguyên

| Tính năng | Đánh giá |
|-----------|----------|
| REST RPM guard | Xuất sắc — loại bỏ false pulse khi mọi output OFF |
| Dry-start khi EGT OPEN | Đúng và an toàn — chạy starter, tắt nhiên liệu/van |
| `egtAllowsFuelIncrease()` lookahead 3s | Logic rõ ràng, đúng |
| ACCEL_TO_IDLE timeout 20s | Cần thiết, được implement tốt |
| Cooldown với starter chạy | Đúng — làm mát sau stop |
| Glow preheat trước khi có nhiên liệu | Đúng (SPINUP_PREHEAT → ignCmd=true trước fuel) |
| Pump calibration table | Khớp chính xác với đo thực tế |
| ISR `IRAM_ATTR` | Đúng — ISR trong IRAM |
| Button 2-step ARM→START | An toàn |

---

## 📊 Điểm An Toàn Tổng Thể (lúc review ban đầu, trước khi fix)

| Hạng mục | Điểm |
|----------|------|
| Logic state machine | 8/10 |
| EGT safety guards | 7/10 *(vấn đề #1 Web UI)* |
| RPM noise handling | 8/10 *(vấn đề #3, #8)* |
| Fuel control safety | 9/10 |
| Serial/Web interface safety | 6/10 *(vấn đề #2, #5)* |
| **Tổng** | **7.5/10** |

**Cập nhật**: cả 8 vấn đề trên đã được fix (xem checklist bên dưới) — bảng
điểm này chỉ còn giá trị lịch sử, không phản ánh trạng thái code hiện tại.

---

## 🚀 Quyết Định: Có Upload ESP32 Được Không?

**YES — đã fix toàn bộ 8/8 vấn đề vòng 1** (trước đây có 4 ràng buộc vận
hành tạm thời cho #1/#2/#5/#6, nay không còn cần thiết vì các lỗi gốc đã
được fix trong code thay vì né bằng quy trình vận hành).

---

## 📋 Fix Priority

```
Fix ngay (trước lab test):
[x] #5 Non-blocking serial command buffer

Fix trước engine test thực sự:
[x] #1 EGT guard trong stage2Off()
[x] #2 Require clearAbort trước startidle
[x] #3 resetRpmStats() trước wet start spin-up

Fix sau (ít quan trọng hơn):
[x] #4 Reset gradientCps khi EGT invalid
[x] #6 SD slot overflow handling — không còn ghi header mới đè vào ECU999.CSV khi đầy 1000 slot
[x] #7 Single SPI read cho readCelsius + readError — đổi thứ tự đọc readError() trước
[x] #8 Dùng stddev thay max-min cho jitter — coefficient of variation, không bị 1 outlier làm sai
```

**Tất cả 8 lỗi vòng 1 đã fix (2026-07-16).** Compile sạch (`g++ -fsyntax-only
-Wall -Wextra`, không warning).

---

# 🔁 Review Vòng 2 (2026-07-16) — 9 lỗi mới, đã fix toàn bộ

Sau khi fix 8 lỗi vòng 1, chạy thêm một vòng review sâu (8 finder angles +
verify) và tìm được **9 lỗi mới**. Tất cả đã được fix và firmware đã
compile sạch (`g++ -fsyntax-only -Wall -Wextra`, không warning).

## 🔴 CRITICAL — An toàn nhiên liệu

| # | Vị trí | Lỗi | Cách fix |
|---|--------|-----|---------|
| 1 | `pumptest` / `test pump` | Mở van + bơm nhiên liệu khi ở MODE_ABORTED mà không kiểm tra EGT → có thể phun nhiên liệu vào buồng đốt còn nóng sau abort OVER_TEMP | Thêm `fuelCommandBlockedByHotEgt()`: chặn khi EGT đọc được và > `cooldownTargetC` |
| 2 | `checkFailures()` | Guard RPM_SIGNAL_LOST bỏ sót `ST_INTRO_FUEL`/`ST_POST_IGNITION_HEAT` → mất tín hiệu RPM khi van nhiên liệu đang mở không bị abort trong tối đa 6s | Guard mới bao trùm cả `MODE_STARTING` (mọi stage có nhiên liệu) |

## 🟠 HIGH — Độ tin cậy RPM & state machine

| # | Vị trí | Lỗi | Cách fix |
|---|--------|-----|---------|
| 3 | `rpmISR()` | Glitch bị reject vẫn cho glitch kế tiếp lọt qua filter → phantom RPM → có thể kích OVERSPEED giả | Thêm **adaptive mask**: reject cạnh đến sớm hơn ½ chu kỳ hợp lệ gần nhất (turbine không thể tăng đôi tốc độ trong 1 vòng) |
| 4 | `updateRpm()` | `nowUs = micros()` lấy trước `noInterrupts()` → ISR chen giữa gây wraparound uint32 → `signalRecent=false` giả → abort RPM_SIGNAL_LOST oan | Dời `micros()` ra **sau** critical section (đảm bảo `nowUs >= lastPulseUs`) |
| 5 | `egtSafeForAbortClear()` | Cảm biến EGT hỏng vĩnh viễn (`egt.ok=false`) chặn TẤT CẢ đường thoát MODE_ABORTED → kẹt cứng, phải cúp nguồn | Thêm `egtAllowsDeliberateAbortClear()` + lệnh `clearabort force` (chỉ dry-start sau đó) |
| 6 | `beginAutoIdle()` | Không xóa `activeTest`/`activeTestEndMs` → timer test đang chạy bắn giữa chu trình start thật, dừng starter | Xóa `activeTest = TEST_NONE; activeTestEndMs = 0;` khi bắt đầu start |
| 7 | `ST_INTRO_FUEL` dòng 1542 | Check NO_RPM_RISE (`fuelConfirmTimeoutMs`=10s) là **dead code** vì NO_IGNITION (6s) luôn abort trước | Chuyển check sang neo theo `ignitionDetectedMs`, chạy trong POST_IGN + ACCEL_TO_IDLE (`checkPostIgnitionRpmRise()`) |

## 🟡 MEDIUM — Hiệu năng / độ bền

| # | Vị trí | Lỗi | Cách fix |
|---|--------|-----|---------|
| 8 | `sdAppendLine()` | SD open/close đồng bộ mỗi event → block main loop 10–100ms trên thẻ chậm, trễ `checkFailures()` | Hàng đợi SD event: snapshot lúc phát sinh, ghi (blocking) trong loop **sau** safety check |
| 9 | `webStatusJson()` | ~25 lần `String +=` không `reserve()` → phân mảnh heap mỗi 700ms poll → nguy cơ Guru Meditation | Thêm `s.reserve(896)` |

## ✅ Kết luận vòng 2

- Firmware **compile sạch** với mock ESP32/Servo/MAX31855/WiFi/WebServer/SD.
- Đã trace toàn bộ luồng: `WAITING → PURGE → SPINUP_PREHEAT → INTRO_FUEL
  (đánh lửa) → POST_IGNITION_HEAT → ACCEL_TO_IDLE → IDLING`. Các fix
  **không phá vỡ** luồng chạy tới đánh lửa và idle.
- Các interlock để chạy wet start thật: `arm2` → `autostart on` → chạy đủ
  Test Wizard + `confirmkill` → `startidle`.

**Còn lại (tùy chọn, không chặn test)**: dry-start hiện bỏ qua toàn bộ
checklist kể cả `TEST_KILL` — cân nhắc vẫn yêu cầu xác nhận kill switch cho
dry-start vì starter vẫn quay trục thật (rủi ro thấp: dry-start không có
nhiên liệu/lửa).

---

# 🔁 Review Vòng 3 (2026-07-16) — So sánh với Rev11/Rev12_TC10, 2 lỗi đã fix

So sánh cấu trúc + an toàn với 2 bản firmware tham khảo
(`REFERENCES/Firmware/Rev11`, `REFERENCES/Firmware/Rev12_TC10`). Kết luận
chung: firmware hiện tại **an toàn hơn** cả 2 bản tham khảo ở mọi interlock
cốt lõi (abort cứng, cooldown thật, ARM/START 2 bước, RPM noise
classification, Test Wizard checklist). Tuy nhiên tìm được 2 khoảng trống
so với Rev11/12, cả 2 đã được fix:

| # | Vị trí | Lỗi | Cách fix |
|---|--------|-----|---------|
| 1 | Không có tương đương RC-signal-loss failsafe của Rev11/12 | `throttlePct` là giá trị chốt qua lệnh Serial/Web rời rạc; nếu mất kết nối Web UI/Serial giữa lúc `MODE_IDLING`/`MODE_OPERATING`, động cơ chạy mãi ở ga cuối cùng không ai giám sát | Thêm **comm watchdog**: `lastOperatorLinkMs` cập nhật ở mọi lệnh (`handleCommand()`) và mỗi lần Web UI poll `/api` (tự động mỗi 700ms khi tab mở). `checkFailures()` tự abort `COMM_TIMEOUT` nếu quá `cfg.commTimeoutMs` (mặc định 8000ms, chỉnh qua `set commtimeout <3000..60000>`) không có tín hiệu nào trong lúc IDLING/OPERATING. Tắt qua `set commwatchdog off` yêu cầu `arm2` trước (debug only) |
| 2 | `abortAll()` → `enterCooldown()` | Log snapshot lúc abort bị ghi **sau khi** `enterCooldown()` đã zero hóa `fuelTargetUs`/`pumpUs`/`ignCmd`/`startUs` → SD/event log thể hiện giá trị đã an toàn hóa, không phải giá trị thật lúc lỗi xảy ra, làm giảm giá trị chẩn đoán sự cố | `abortAll()` gọi `addLog("ABORT_SNAPSHOT reason=...")` **trước** khi gọi `enterCooldown()`, chụp đúng RPM/EGT/fuel/outputs tại thời điểm lỗi |

**Verify**: compile sạch (`g++ -fsyntax-only -Wall -Wextra`, mock ESP32/Servo/
MAX31855/WiFi/WebServer/SD, không warning).

---

# 🔧 Cập nhật vai trò Valve1/Valve2 (2026-07-16) — theo manual EnJet E86/G3

Đối chiếu `REFERENCES/Documentation/Engine_Manual_NewerModel.pdf` (EnJet G3
Series manual, có bảng thông số E86) xác nhận động cơ dùng **2 valve dầu**
(không phải gas): **Start solenoid valve** (mạch dầu khởi động riêng) +
**Main oil valve** (mạch dầu chính, luôn mở khi bơm chạy — theo mô tả
"Oil Pump Test" tự động liên kết mở Main valve trong manual).

Trước đây `fuelValvesAuto()` mở/đóng cả 2 valve đồng thời, không phân biệt
vai trò. Đã cập nhật:

```cpp
void fuelValvesAuto(bool on) {
  valve2Cmd = on;                              // Main: mở bất cứ khi nào có lệnh nhiên liệu
  valve1Cmd = on && (ecuMode == MODE_STARTING); // Start: chỉ mở trong lúc MODE_STARTING
}
```

- **VALVE1 (Start)** chỉ mở trong `ST_INTRO_FUEL → ST_POST_IGNITION_HEAT →
  ST_ACCEL_TO_IDLE` (toàn bộ `MODE_STARTING`), tự động đóng ngay khi chuyển
  sang `MODE_IDLING`.
- **VALVE2 (Main)** giữ hành vi cũ — mở bất cứ khi nào `fuelValvesAuto(true)`
  được gọi, bất kể mode (khớp `pumptest`/Test Wizard pump-prime = "Oil Pump
  Test" của Enjet, chỉ mở Main).
- Checklist đổi tên hiển thị `VALVE_1`→`VALVE1_START`, `VALVE_2`→`VALVE2_MAIN`
  để rõ vai trò trên Serial/Web UI.
- Đã rà soát toàn bộ điểm gọi `valve1Cmd`/`valve2Cmd` (rest-guard, fueled
  check trong `checkFailures()`, log) — không có điểm nào phụ thuộc hành vi
  "2 valve luôn giống nhau" nên không có tác dụng phụ.

**Lưu ý còn để ngỏ**: giả định trên dựa vào tài liệu chính hãng, chưa xác
nhận vật lý E86 đời cũ của người dùng có đúng 2 mạch dầu tách biệt hay
không — nên đối chiếu lại đường ống thật trước khi test có nhiên liệu.

**Verify**: compile sạch (`g++ -fsyntax-only -Wall -Wextra`, không warning).

---

# 🔧 ESP32Servo → LEDC + Starter Kick + Web UI cho TEST_STARTER (2026-07-17)

## Bối cảnh

Test bench với nguồn 12V 2A: starter khựng/giật giật rồi dừng dù firmware
vẫn gửi PWM đều đặn. Ban đầu chẩn đoán là thiếu dòng cấp nguồn (đúng, xem
mục "Nguồn & Pin" trong `COMMISSIONING_GUIDE.md`), nhưng người dùng thử
nghiệm 1 sketch riêng dùng LEDC PWM trực tiếp (không qua thư viện
`ESP32Servo`) — **chạy êm với CÙNG một nguồn 12V 2A**. Điều này cho thấy
`ESP32Servo` là một nguyên nhân riêng biệt, độc lập với nguồn điện — xem
chi tiết kỹ thuật trong `CLAUDE.md` ở gốc repo.

## Thay đổi

| # | File | Thay đổi |
|---|------|---------|
| 1 | `ECU_TestV1_EGT_DRY_START_PATCH.ino` | Bỏ `ESP32Servo`, chuyển pump+starter ESC sang LEDC PWM trực tiếp (`escAttach()`/`escWriteUs()`, tương thích cả ESP32 Arduino core 2.x và 3.x qua `ESP_ARDUINO_VERSION_MAJOR`) |
| 2 | `ECU_TestV1_EGT_DRY_START_PATCH.ino` | Thêm **starter kick**: `cfg.starterKickUs`/`cfg.starterKickMs` — xung cao ngắn (mặc định 1300us/300ms) ngay đầu `ST_PURGE` trước khi hạ về `starterPurgeUs`, giúp starter có cơ cấu Bendix/clutch ăn khớp dứt khoát. Lệnh mới: `set starterkickus`, `set starterkickms` |
| 3 | `ECU_TestV1_EGT_DRY_START_PATCH.ino` | Web UI dashboard: thêm mục "Starter Kick" (chỉnh kickUs/kickMs) và "Starter Manual Test" (chạy `starttest <us> <ms>` từ trình duyệt) |
| 4 | `TEST/TEST_STARTER/TEST_STARTER.ino` | Cùng 3 thay đổi trên: LEDC thay ESP32Servo, thêm kick (`kickus`/`kickms`), và thêm **Web UI đầy đủ** (SoftAP `TEST_STARTER`/`test1234`, dashboard + toàn bộ lệnh qua nút/ô nhập) — cho phép test không cần Serial Monitor |

**Verify**: compile sạch cả 2 file (`g++ -fsyntax-only -Wall -Wextra`, không warning).

**Lưu ý**: `starterKickMs=0` tắt hoàn toàn tính năng kick nếu không cần.

---

# 🔧 Gỡ starter kick + tăng giá trị test (2026-07-18)

Theo yêu cầu: **bỏ hẳn tính năng starter kick** (không cần công đoạn này nữa)
và nâng giá trị test.

| # | File | Thay đổi |
|---|------|----------|
| 1 | `ECU_TestV1_EGT_DRY_START_PATCH.ino` | Gỡ `cfg.starterKickUs`/`cfg.starterKickMs`, helper `starterKickOrSteady()`, 2 lệnh `set starterkickus`/`set starterkickms`, mục Web UI "Starter Kick", và dòng in trong printConfig/printHelp. `ST_PURGE` (cả start thật lẫn dry-start) giờ dùng thẳng `starterPurgeUs` |
| 2 | `ECU_TestV1_EGT_DRY_START_PATCH.ino` | **starterSpinUs 1150 → 1200** (test wizard "Starter" + spin thật + default Web UI "Starter Manual Test"). **introFuelUs 1160 → 1210** (pump prime test + intro fuel thật). Nới trần bench `pumptest` 1175 → **1225µs** để đạt mức mới |
| 3 | `TEST/TEST_STARTER/TEST_STARTER.ino` | Gỡ toàn bộ kick (globals, `applyPwm`/`setPwm`, lệnh `kickus`/`kickms`, trường JSON, mục Web UI, docstring). Default PWM Web UI 1150 → 1200 |

> ⚠️ **ĐÃ THAY ĐỔI SAU (superseded)**: hàng #2 để `introFuelUs=1210` gây `introFuelUs > idleFuelUs`
> (khởi động quá giàu). Xem mục **2026-07-18 (review toàn project)** bên dưới: `introFuelUs`
> đã khôi phục về **1160** (< idle) và giá trị pump test **1210** tách sang biến riêng `pumpTestUs`.

**Lưu ý (đã lỗi thời — xem ghi chú trên)**: `introFuelUs`(1210) từng > `idleFuelUs`(1175)
theo quyết định "đổi cả config chung"; nay đã tách `pumpTestUs`.

**Không hard-code — chỉnh runtime**: các mức PWM starter/fuel giờ đổi được lúc
chạy, không cần build lại firmware:
- Lệnh mới: `set purgeus <us>`, `set spinus <us>`, `set assistus <us>` (1000..1500)
  cho starterPurge/Spin/Assist. `set intro`/`set idleus`/`set maxus` đã có sẵn.
- Web UI: thêm panel **"Starter & Fuel PWM"** với ô nhập cho purge/spin/assist +
  intro/idle/max, mỗi ô có nút Set gọi lệnh tương ứng.
- `printConfig` in thêm dòng `starter purge/spin/assist us=...`.
- Giá trị trong Config struct chỉ còn là **mặc định lúc khởi động**, ghi đè
  được bất cứ lúc nào qua Serial hoặc trình duyệt.

**Verify**: compile sạch cả 2 file (`g++ -fsyntax-only -Wall -Wextra`, không
warning) + mô phỏng chèn auto-prototype kiểu Arduino cho firmware chính (OK).

---

# 🔎 Review toàn project + fix an toàn (2026-07-18)

Review 4 mảng bằng agent song song (state machine, cảm biến/ISR/EGT/fuel,
Web UI/SD/command, TEST_STARTER). Xác nhận **không** có đọc ISR uint64 hở,
**không** chia 0, LEDC duty đúng, gỡ kick sạch. Đã fix các lỗi sau:

**HIGH**
- **#1/#2 Web UI default lệch config**: `Max EGT` hiển thị 780 (config 680),
  `Idle RPM` 32000 (config 42000) → bấm Set là âm thầm đổi ngưỡng bảo vệ.
  Fix: sửa default khớp config **và** thêm field `cfg*` vào `/api`, JS `setInp()`
  tự nạp mọi ô tune từ config mỗi poll (không ghi đè khi đang gõ).
- **#3 `stop` từ ABORTED** xóa lý do lỗi + quay starter, rò ABORTED→WAITING bỏ
  qua interlock. Fix: `requestStop()` chỉ tác dụng khi STARTING/IDLING/OPERATING.
- **#4 Start bằng nút bị comm-watchdog abort ngay khi vào IDLING**. Fix: cờ
  `runStartedByButton` (bỏ qua watchdog cho run bằng nút; re-engage khi có lệnh
  remote) + reset `lastOperatorLinkMs` trong `beginAutoIdle`.
- **#5 Lệnh `set` an toàn không bị chặn khi chạy**. Fix: chặn tune PWM/limit
  (intro/idleus/maxus/pumptestus/purgeus/spinus/assistus/idlerpm/maxrpm/rpmtol/
  maxegt) — chỉ nhận khi WAITING/ABORTED.

**MEDIUM**
- **#6/#7 intro>idle + pump test ~3x fuel**: **tách** `pumpTestUs`(1210, bench
  prime) khỏi `introFuelUs`(khôi phục 1160 < idle). `test pump` dùng `pumpTestUs`;
  thêm `set pumptestus` + ô Web UI; sửa comment/thông báo flow.
- **#11 `off` từ ABORTED** rời ABORTED không set ack. Fix: `stage2Off()` ở ABORTED
  chỉ re-assert safe, yêu cầu `clearabort`.
- **#9 SD telemetry blocking**: `sdAppendLine` đếm lỗi liên tiếp, sau 5 lần đặt
  `sdOk=false` (ngừng block loop khi rút thẻ). *(Slow-card vẫn cần task riêng.)*

**LOW**
- **#15** nới `webStatusJson reserve` 896→2048, `sdCsvLine` 240→320.
- **#16** `valve1/valve2 on` auto-off sau 10s (`VALVE_TEST_TIMEOUT_MS`).

**Còn để lại (chưa fix, low)**: #10 watchdog thỏa mãn bởi auto-poll, #12
overspeed/flameout gate sau NOISY (được RPM_SIGNAL_LOST cứu), #14 SoftAP mật khẩu
yếu, #17 ABORTED chưa chặn EGT nóng cho ign/starter, #18 không phát hiện EGT
đóng băng, #19 micros()==0 sentinel, #20 jsonEscape control chars.

**Verify**: compile sạch (`g++ -fsyntax-only -Wall -Wextra`) + mô phỏng
auto-prototype Arduino cho firmware chính.

---

# 🔎 Quét toàn project vòng 2 + fix logic điều khiển (2026-07-18b)

Quét lại bằng 3 agent (hồi quy PR #17, correctness mới, docs↔code). Xác nhận PR #17
đúng, không hồi quy; TEST_STARTER không bug. Fix thêm các lỗi logic điều khiển:

**Logic điều khiển (#1–4)**
- **#1** Soft cut quá nhiệt trước đây chỉ ~2µs/s (không kịp) → khi `egtRequestsFuelCut()`
  giờ hạ `pumpUs` theo bước `fuelCutStepUs` ở nhịp decel nhanh (~40µs/s), vẫn có hard
  OVER_TEMP làm net cuối.
- **#2** `updateOperating`: governor full-throttle nhắm `maxRpm - rpmTolerance` (không
  còn tự kích OVERSPEED ở 100% throttle).
- **#3** RPM-loss khi IDLING/OPERATING dùng `FUELED_RPM_LOSS_TIMEOUT_MS=400ms` (thay 1s)
  → cắt nhiên liệu nhanh sau flameout thật.
- **#4** Grace flameout neo vào `runningSinceMs` (idle ổn định đầu tiên), không reset khi
  gạt throttle IDLING↔OPERATING.

**Tuning/robustness (#5–8)**
- **#5** Trong `MODE_STARTING`, bỏ chặn theo gradient/look-ahead 3s (rise lúc light-off là
  bình thường) — chỉ còn `maxEgtC` tuyệt đối + look-ahead ngắn của abort. Thêm `set maxgrad`.
- **#6** RPM_SIGNAL_LOST lúc STARTING chỉ dựa vào "có xung gần đây" (recency), không dựa
  vào phân loại NOISY → nhiễu ign lúc light-off không abort oan; sensor chết vẫn bắt được.
- **#7** Comm watchdog phủ cả `MODE_STARTING` khi đã có nhiên liệu (không phủ PURGE/SPINUP khô;
  run bằng nút vẫn miễn).
- **#8** OVER_TEMP abort thêm look-ahead ngắn `EGT_ABORT_LOOKAHEAD_S=0.2s` bù trễ ~1 mẫu EGT.

**Docs/cosmetic**: bỏ `ESP32Servo` khỏi README; sửa số dòng/ngày; ghi chú superseded cho
mục introFuelUs=1210; zero timer valve trong forceSafeOutputs; nới reserve JSON 2048→2560.

**Verify**: compile sạch (`g++ -fsyntax-only -Wall -Wextra`, không warning) + mô phỏng
auto-prototype Arduino (OK).

> **Cần validate trên động cơ thật**: tốc độ cắt nhiên liệu #1, ngưỡng governor #2, các
> giá trị timeout/gradient #3/#5/#8 nên được kiểm chứng bằng lần chạy thật và tinh chỉnh
> qua `set ...`.

---

# 🔻 Xử lý nốt các mục LOW (2026-07-18c)

- **#17** Chặn `ignpulse` và `valve1/valve2 on` khi EGT còn nóng (`fuelCommandBlockedByHotEgt`)
  — không cho châm lửa/mở van vào lõi nóng ở ABORTED. `starttest` (chỉ starter, giúp làm mát) giữ nguyên.
- **#18** Cảnh báo **log-only** (không abort) khi EGT đứng yên byte-for-byte > `EGT_STUCK_WARN_MS`
  lúc đang cháy → gợi ý thermocouple đóng băng (loại lỗi mà fault open/short không bắt được).
- **#19** ISR ép `micros()` khác 0 (bỏ va chạm sentinel "chưa có cạnh" mỗi ~71 phút) — cả 2 firmware.
- **#20** `jsonEscape` xử lý mọi ký tự control (`\r`/`\t`/`\u00XX`) → JSON không vỡ.
- **#14** Thêm ghi chú bảo mật + đánh dấu `WEB_PASS` cần đổi trước khi chạy thật (cả 2 firmware);
  không tự đổi mật khẩu (giữ credential người dùng đang dùng).
- **#10** `/api` chỉ tính là "operator hiện diện" khi tab đang hiển thị (client gửi `act=0` khi
  ẩn/khoá màn hình) → phiên bỏ đi sẽ time-out thay vì bị auto-poll giữ sống mãi. Tương thích ngược.
- **#12** Không sửa: overspeed/flameout khi NOISY đã được `RPM_SIGNAL_LOST` bao (NOISY + có nhiên
  liệu → abort cắt nhiên liệu), nên đã an toàn.

**Verify**: compile sạch (`g++ -fsyntax-only -Wall -Wextra`) + mô phỏng auto-prototype Arduino.

---

# 🔎 Quét toàn project vòng 3 (2026-07-18d)

Quét lại bằng 2 agent (correctness + hồi quy toàn firmware; TEST_STARTER + đối chiếu
doc↔code). Xác nhận: **các fix vòng 1–2 đúng, không hồi quy**; **TEST_STARTER không bug**;
`CODE_ARCHITECTURE.md` khớp code (chỉ 3 chỗ chữ nghĩa nhỏ đã vá). Fix 3 điểm mới:

- **#1 `set cooldownms` hỏng**: `parseTwoInts` cắt theo dấu cách đầu tiên nhưng lệnh có
  prefix 3 từ → `minMs` parse nhầm thành 0 → luôn báo lỗi. Sửa: parse riêng 2 số sau prefix.
- **#2 `off`/SAFE-OFF lúc COOLDOWN-sau-abort** lọt sang WAITING (xóa lý do lỗi, bỏ qua
  `clearabort`, cắt luôn airflow làm mát). Sửa: `stage2Off()` bỏ qua khi
  `MODE_COOLDOWN && cooldownAfterAbort` → để cooldown chạy hết vào ABORTED.
- **#3 RPM_SIGNAL_LOST abort oan động cơ đang chạy** (có sẵn): NOISY do `rejected≥3` tuyệt
  đối trong 1 cửa sổ 100ms có thể tắt máy khoẻ. Sửa: **debounce** NOISY-nhưng-còn-xung ở
  IDLING/OPERATING — chỉ abort nếu kéo dài ≥ `RPM_NOISY_ABORT_MS` (300ms); mất xung thật
  (recency 400ms) vẫn abort ngay.

**Verify**: compile sạch (`g++ -fsyntax-only -Wall -Wextra`) + mô phỏng auto-prototype Arduino.

---

# 🔎 Review GATE trước merge — vòng 4 (2026-07-18e)

Quét thật kỹ bằng 4 agent (verify sâu 3 fix vòng 3; audit toàn bộ state machine/interlock;
sensor/ISR/toán/concurrency; command/Web/SD). Xác nhận: 3 fix vòng 3 **đúng**; re-arm
interlock **sạch, không còn rò**; toán học/ISR/atomicity/chia-0/LEDC/parse/SD **đúng**;
TEST_STARTER **sạch**. Fix 8 điểm mới (gồm 1 hồi quy):

- **A** (an toàn) Test Wizard `test ign/starter_ign/valve1/valve2` giờ chặn EGT nóng
  (`fuelCommandBlockedByHotEgt`) như các lệnh trực tiếp; `test starter` vẫn cho (làm mát).
- **B** (hồi quy do #7 vòng 2) Comm watchdog **bỏ phủ MODE_STARTING** → start-bằng-Serial
  không còn bị COMM_TIMEOUT oan giữa chừng; fuel-unattended lúc start vẫn được stage timeout chặn.
- **G** (an toàn) Lệnh reset RPM (`rpmreset`/`set rpmfilter`/`set rpmedge`, cả nút Web) không
  còn gây `RPM_SIGNAL_LOST` oan khi đang chạy: thêm **grace** sau `resetRpmStats()`
  (`rpmStatsResetAtMs`) trong checkFailures.
- **D** SPINUP starter-prove (2 chỗ) dùng **recency** thay `rpmMeasurementUsable()` → EMI
  igniter/starter không abort `NO_STARTER_RPM` oan (đồng bộ với checkFailures STARTING).
- **H** `set idlerpm` không cho vượt `maxRpm-5000` (giữ invariant idle<max, tránh governor
  nhắm trên OVERSPEED).
- **I** `set ppr` giờ chỉ nhận ở WAITING/ABORTED (đổi mid-run rescale RPM → FLAMEOUT/OVERSPEED oan).
- **E** `classifyRpmNoise`: xung mồi đầu tiên (raw==1, accepted==0) không còn bị coi NOISY (đổi `raw>0`→`raw>1`).
- **F**/**J** vá sentinel `micros()==0` trong updateRpm; nới reserve JSON 2560→3072.

**Chấp nhận (không sửa)**: `off` lúc soft-stop cooldown cắt airflow sớm (thao tác chủ ý,
restart vẫn chặn EGT); phơi nhiễm overspeed ≤300ms khi NOISY (đánh đổi có chủ đích);
`checkPostIgnitionRpmRise` dùng usable (cửa sổ 10s che, rủi ro thấp); millis rollover ~49.7 ngày.

**Verify**: compile sạch (`g++ -fsyntax-only -Wall -Wextra`, không warning) + mô phỏng
auto-prototype Arduino (OK).

---

**Người review**: Code Review Agent (automated)  
**Phiên bản firmware**: ECU_TestV1_EGT_DRY_START_PATCH  
**Lần cập nhật**: 2026-07-18e (review GATE vòng 4 trước merge: 8 fix gồm hồi quy watchdog)
