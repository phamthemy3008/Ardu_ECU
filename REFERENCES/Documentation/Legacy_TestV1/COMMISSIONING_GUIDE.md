# Hướng Dẫn Toàn Diện: Upload, Hiệu Chỉnh & Commissioning ECU

**Firmware**: `ECU_TestV1_EGT_DRY_START_PATCH.ino`
**Board**: ESP32 DevKit V1 / NodeMCU-32S
**Đối tượng**: Từ upload firmware lần đầu tới chạy thật có tăng throttle, đi từng bước an toàn.

> Nguyên tắc xuyên suốt: **không nhảy cóc bước**. Mỗi giai đoạn dưới phải
> PASS hết rồi mới sang giai đoạn kế tiếp. Nếu bất kỳ bước nào FAIL — dừng,
> tìm nguyên nhân, không ép qua bằng debug override.
>
> Tài liệu này gộp toàn bộ các hướng dẫn trước đây
> (Upload & Test, Pre-Engine Test, DSO152 RPM Calibration, Commissioning)
> thành **1 file duy nhất** để không phải lật qua nhiều tài liệu.
>
> **Điều khiển & test hoàn toàn qua Web UI (không cần terminal).** Kể từ bản
> hiện tại, mọi thao tác điều khiển, Test Wizard, manual test (starter / pump /
> glow / valve) và toàn bộ tham số tune (RPM, PWM, EGT, timing, cooldown,
> interlock) đều có nút/ô nhập trên giao diện web `http://192.168.4.1`
> (SoftAP `ECU_TestV1` / `admin1234`). Các lệnh Serial vẫn còn hoạt động như
> phương án dự phòng, nhưng không bắt buộc dùng.
>
> Giao diện web chia **4 tab**:
> - **▶ Run** — điều khiển (ARM / START IDLE / AutoStart / SOFT STOP / SAFE OFF /
>   CLEAR ABORT / RPM RESET), chế độ EGT, và các thẻ trạng thái chi tiết.
> - **🧪 Testing** — Test Wizard (9 bước + checklist) và Manual Actuator Test
>   **có thời lượng** (starter/pump chạy đúng số ms nhập vào rồi tự tắt, ignpulse
>   glow có giới hạn thời gian) — dùng để test nhanh, có kiểm soát thời gian.
> - **🔧 Manual** — điều khiển tay **giữ nguyên tới khi tự tắt** (không có thời
>   lượng): starter (`startmanual <us>`/`off`), pump (`pumpmanual <us>`/`off`,
>   tự mở Valve 1), glow (`ign on`/`off`), Valve 1, Valve 2 — kèm nút
>   **"🛑 SAFE OFF — TẮT HẾT NGAY"** và vài thẻ trạng thái (RPM/EGT/Starter/
>   Pump/IGN/Valve) ngay trên tab để không phải chuyển qua Run. Dùng khi cần
>   giữ 1 cơ cấu chạy liên tục để quan sát (đo dòng, đo nhiệt, canh chỉnh...).
> - **⚙ Settings** — toàn bộ tham số tune + lưu/nạp config, mục **"Advanced"**
>   (timing/cooldown/interlock) mở/gập được.
>
> Phía trên luôn hiển thị **2 đồng hồ (gauge) RPM & EGT** và dải chip trạng thái
> (MODE / STAGE / ARM / AUTO / NOISE / CHECK / ABORT) đổi màu theo trạng thái.
>
> **Lưu config vào thẻ SD (cấu hình 1 lần):** trong tab **Settings**, bấm
> **"💾 Lưu config vào SD"** để ghi toàn bộ tham số tune ra `/ECUCFG.TXT`. Lần
> sau bật máy, firmware **tự nạp lại** file này lúc boot (mọi giá trị được kẹp
> trong khoảng hợp lệ khi nạp). Nút **"↻ Nạp lại từ SD"** nạp lại ngay không cần
> khởi động lại. Vì an toàn, `autoStart`, throttle và web KHÔNG được lưu — luôn
> khởi động ở trạng thái an toàn. (Lệnh Serial tương đương: `savecfg` / `loadcfg`.)

---

## ⚠️ Cảnh Báo An Toàn Chung

- Luôn test trong không gian thoáng khí
- Tháo nhiên liệu ra khỏi hệ thống khi test điện tử/component
- Giữ khoảng cách an toàn khi động cơ/starter quay
- Có người thứ 2 giám sát khi test engine thực có nhiên liệu
- Chuẩn bị bình chữa cháy **CO2** gần tầm tay (không dùng bột khô)
- Kill switch vật lý (NC) phải lắp và test cơ học độc lập — không dựa vào firmware

---

## Yêu Cầu Trước Khi Bắt Đầu

### Hardware
- [ ] ESP32 DevKit V1 hoặc NodeMCU-32S
- [ ] Cáp USB (USB-A to Micro-USB)
- [ ] Máy tính có Arduino IDE
- [ ] MAX31855 Thermocouple Module + Thermocouple loại K
- [ ] RPM Sensor (KMZ10A AMR + mạch khuếch đại/so sánh — xem Giai đoạn 2)
- [ ] 2× ESC (Pump & Starter)
- [ ] 2× Solenoid Valve (Start + Main, xem `CODE_REVIEW_FINDINGS.md`)
- [ ] Glow plug/igniter + driver
- [ ] Micro SD Card (FAT32 format)
- [ ] Nguồn cấp đủ dòng cho starter/pump/glow (xem phần Nguồn & Pin bên dưới)

### Software
- [ ] Arduino IDE 1.8.19+ hoặc 2.x
- [ ] ESP32 Board Support Package
- [ ] Library: **Adafruit MAX31855**
  (không cần ArduinoJson hay ESP32Servo — firmware dùng LEDC PWM trực tiếp
  cho ESC, xem `CLAUDE.md` ở gốc repo)

---

## Cài Đặt Arduino IDE & Upload Firmware

### Bước 1: Cài Arduino IDE
1. Tải từ https://www.arduino.cc/en/software, cài đặt theo hướng dẫn

### Bước 2: Thêm ESP32 Board
1. **File → Preferences** → thêm URL Board Manager:
   `https://dl.espressif.com/dl/package_esp32_index.json`
2. **Tools → Board → Board Manager** → tìm "ESP32" (tác giả Espressif Systems) → cài
3. Chọn board: **ESP32 Dev Module**

### Bước 3: Cài Library
**Sketch → Include Library → Manage Libraries** — tìm và cài:

| Library | Tác giả |
|---------|---------|
| Adafruit MAX31855 | Adafruit Industries |

> Không cần cài ESP32Servo — ESC (pump/starter) dùng LEDC PWM có sẵn trong
> ESP32 Arduino core. Xem `CLAUDE.md` ở gốc repo.

### Bước 4: Cấu Hình Board & Upload
```
Board:           ESP32 Dev Module
Upload Speed:    115200
Flash Size:      4MB
Port:            COM3 (Windows) hoặc /dev/ttyUSB0 (Linux)
```
Mở `PROJECT_CURRENT/Firmware/ECU_TestV1_EGT_DRY_START_PATCH/ECU_TestV1_EGT_DRY_START_PATCH.ino`
→ Click **Upload** (hoặc Ctrl+U) → chờ "Done uploading" (~30-60s).

**Nếu upload thất bại** ("Failed to connect to ESP32: Timed out"):
1. Giữ nút **Boot** trên board trong lúc upload, thả ra khi thấy "Connecting..."
2. Thử lại, hoặc giảm Upload Speed xuống 9600 để chẩn đoán
3. Kiểm tra driver CH340 đã cài (Windows): http://www.wch.cn/download/ch341ser_exe.zip

### Bước 5: Mở Serial Monitor
**Tools → Serial Monitor** (Ctrl+Shift+M): Baud **115200**, Line ending **Newline**.

Output kỳ vọng ngay sau boot:
```
Test ECU V1 WebUI/TestWizard booted.
MAX31855 begin() = OK (hoặc CHECK_WIRING)
RPM edge = RISING
SD_LOG: OK file=/ECU000.CSV (hoặc FAIL file=...)
Outputs safe. Auto-start disabled. Type help.
```

**Nếu Serial Monitor trống**: kiểm tra baudrate = 115200, upload lại sketch, nhấn nút Reset trên board.

---

## Sơ Đồ Đấu Nối GPIO (Pinout)

```
MAX31855 (EGT):    CLK=18   CS=5    DO=19
RPM sensor:        Signal=33
Pump ESC:          Signal=26 (PWM)
Starter ESC:       Signal=25 (PWM)
Valve 1 (Start):   17 (digital)
Valve 2 (Main):    16 (digital)
Ignition/Glow:     32 (digital)
User Button:       22 (digital, active-LOW, pull-up nội bộ)
Status LED:        2 (onboard, active-LOW)
SD Card (SPI):     CS=13  SCK=14  MOSI=23  MISO=27
```

**Kết nối tối thiểu để test không cần động cơ**: ESP32 + MAX31855 + thermocouple
+ RPM sensor + SD card. Không cần ESC/valve/glow để test qua Web UI cơ bản.

---

## Nguồn & Pin — Lưu Ý Bắt Buộc

- **Không cắm thẳng pin LiPo (2S/3S) vào rail 5V logic của ESP32/mạch RPM sensor**
  — rail này cần nguồn **đã ổn áp qua UBEC 5V** riêng, không phải điện áp pin thô.
- **Starter/Pump ESC**: lấy điện trực tiếp từ pin chính — kiểm tra ESC ghi rõ
  dải điện áp hỗ trợ (ví dụ "2S-4S") trước khi chọn pin.
- **Van solenoid + glow plug**: nếu ghi định mức thấp (ví dụ "DC:6V"), phải
  cấp qua UBEC riêng phù hợp dòng (glow plug thường kéo dòng lớn, 3-8A — cần
  UBEC công suất cao hơn UBEC nhỏ dùng cho ESP32/logic).
- **Chọn dòng xả pin (C-rating)** đủ cho tải đỉnh lúc starter cranking + glow
  + bơm chạy cùng lúc — ước tính và kiểm tra thực tế bằng cách đo sụt áp khi
  `test starter_ign` chạy.
- **Nguồn dòng thấp (ví dụ adapter 12V 2A) KHÔNG đủ chạy starter thật** —
  motor sẽ khựng/giật giật/dừng dù firmware vẫn gửi PWM đều đặn (đây là
  triệu chứng thiếu dòng cấp nguồn, không phải lỗi firmware). Dùng ATX PSU
  hoặc bench PSU chịu dòng ≥15-20A, hoặc pin LiPo phù hợp để test thật.

---

## Giai đoạn 0 — Chuẩn bị trước khi cấp điện

### Checklist chung
- [ ] Kill switch vật lý (NC) đã lắp và test cơ học (không dựa vào firmware)
- [ ] Bình lửa CO2 trong tầm với
- [ ] Bàn test cố định động cơ chắc chắn, không ai đứng ngay trục quay
- [ ] Chưa gắn ống dẫn nhiên liệu vào engine thật (test bench trước)
- [ ] Thẻ SD đã format FAT32, gắn vào ECU
- [ ] Firmware mới nhất đã nạp qua Arduino IDE

### Checklist đấu nối vật lý (kiểm tra bằng mắt/đồng hồ trước khi cấp điện)

| Điểm kiểm tra | Yêu cầu |
|---|---|
| GPIO18/5/19 → MAX31855 CLK/CS/DO | Đúng thứ tự, không ngược |
| Thermocouple | Loại K, không phải J hoặc T |
| GPIO33 → RPM sensor | Qua chuỗi khuếch đại/so sánh (xem Giai đoạn 2), không nối thẳng KMZ10A |
| GPIO26/25 → Pump/Starter ESC | Dây tín hiệu PWM, không phải dây nguồn |
| GPIO17/16 → Valve 1/2 | Qua MOSFET/relay driver, không nối thẳng cuộn solenoid |
| GPIO32 → Ignition/Glow | Qua MOSFET driver phù hợp dòng glow plug |
| GPIO22 → User Button | Pull-up, active LOW |
| SD: CS=13 SCK=14 MOSI=23 MISO=27 | Đúng pinout |
| Nguồn ESP32 | 5V ±0.2V qua UBEC riêng, tách biệt nguồn ESC (xem phần Nguồn & Pin) |
| Không ngắn mạch | Đo điện trở GND-VCC khi CHƯA cấp điện |

---

## Giai đoạn 1 — Kiểm tra tĩnh (không cấp nhiên liệu, không quay starter)

Kết nối Serial Monitor 115200 baud hoặc mở Web UI `192.168.4.1`
(WiFi: **ECU_TestV1**, pass: **admin1234**).

| Bước | Lệnh | Kỳ vọng |
|------|------|---------|
| 1.1 | `status` | ECU in ra MODE=WAITING, không lỗi |
| 1.2 | `showcfg` | Xem toàn bộ config mặc định, ghi lại để đối chiếu sau |
| 1.3 | `test egt` | EGT=OK và nhiệt độ phòng (~20-30°C). Nếu FAULT → kiểm tra dây MAX31855 trước khi đi tiếp |
| 1.4 | `test rpm_noise` | RNOISE=CLEAN hoặc NO_SIGNAL (không quay gì cả → NO_SIGNAL là bình thường) |
| 1.5 | `sdstatus` rồi `sdtest` | SD=OK, file ECU0xx.CSV được tạo, event ghi được |

**PASS điều kiện**: cả 5 mục trên không có lỗi. Đây là bước bắt buộc trước
khi chạm vào bất kỳ thứ gì có chuyển động.

**Web UI**: mở dashboard, kiểm tra hiển thị EGT/RPM/Mode=WAITING, Event Log
có dòng boot, Test Wizard tab hiện đủ 9 mục `NOT_RUN`.

---

## Giai đoạn 2 — Hiệu Chỉnh & Xác Nhận Cảm Biến RPM (KMZ10A)

> KMZ10A **không phải Hall sensor thông thường** — đây là cảm biến
> **từ trở anisotropic (AMR)** dạng cầu Wheatstone, output analog vi sai
> mV, cần qua toàn bộ chuỗi khuếch đại + so sánh trước khi thành xung
> digital cho ESP32. Giai đoạn này gồm 2 phần: **A** (oscilloscope DSO152,
> chỉnh phần cứng analog) và **B** (firmware bench, quét toàn dải PWM).

### Sơ Đồ Signal Chain

```
KMZ10A (U17, header 4 chân)
  Pin 1: +V0 → SENSOR_P [TP: U13]   Pin 3: −V0 → SENSOR_N [TP: U12]
  Pin 4: VCC → +5V                   Pin 2: GND → GND
        │ R41/R42 (100Ω) + C19 (1nF lọc vi sai)
        ▼
   INA826 U20 — Gain = 1 + 100k/2.7k ≈ 38×, REF=GND, V+ pin6=VREF(2.5V)
        │ INA_OUT [TP: U3]
        ▼
   C15(10µF) + R35(1M) + R37(4K7) — AC coupling + LPF
        ▼
   LM358 U18 Op-Amp 1 ← RP1 100K (OFFSET)
        ▼
   LM358 U18 Op-Amp 2 ← RP2 100K (GAIN)
        │ LMV358_OUT [TP: U1] (tên net/testpoint trên board, IC thực tế là LM358)
        ▼
   R32(10K) + C2(4.7nF LPF)
        ▼
   LMV393 U19 Comparator (Schmitt, hysteresis R36 470K)
   IN− = VTH_RPM [TP: U21] ← RP3 10K (THRESHOLD)
        │ RPM_OUT [TP: U2] ← ĐIỂM ĐO QUAN TRỌNG NHẤT
        ▼
   J_RPM_OUT1 → cáp ngoài → RPM_PIN_IN1
        │ R10(10K pull-up 3.3V) + R2(1K series) + D1(TVS 3.3V)
        ▼
   GPIO33 (ESP32 NodeMCU-32S)
```

### Bản Đồ Testpoint

| Ký hiệu | Tên | Loại tín hiệu | Dùng cho |
|---------|-----|--------------|---------|
| U15 | TP_GND | 0V reference | Kẹp GND probe suốt quá trình đo |
| U16 | TP_5V | DC +5V | Kiểm tra nguồn mạch analog |
| U14 | TP_VREF | DC ~2.5V | Kiểm tra mid-rail |
| U13/U12 | TP_SENSOR_P/N | Analog mV | Output cầu KMZ10A |
| U3 | TP_INA_OUT | Analog ×38 | Output INA826 |
| U1 | TP_LMV358_OUT | Analog trimmed | Trước comparator |
| U21 | TP_VTH_RPM | DC 0–5V | Ngưỡng comparator (chỉnh RP3) |
| U2 | **TP_RPM_OUT** | **Digital 0/5V** | **Xung RPM cuối — quan trọng nhất** |
| GPIO33 | Trực tiếp | Digital 0/3.3V | Vào ESP32 |

### 3 Trimpot

| Trimpot | Giá trị | Chức năng | Nếu chỉnh sai |
|---------|---------|----------|----------|
| **RP1** | 100K | OFFSET — dịch tâm DC của sóng, không đổi biên độ | Lệch tâm khỏi 2.5V → mất xung ở 1 trong 2 cực nam châm, RPM đọc chỉ bằng nửa thật |
| **RP2** | 100K | GAIN — kéo giãn/co biên độ AC, không đổi tâm DC | Quá thấp → không đủ vượt ngưỡng (NO_SIGNAL); quá cao → clipping, double-pulse, RPM giả cao |
| **RP3** | 10K | THRESHOLD — dịch ngưỡng so sánh VTH_RPM của comparator | Quá thấp → double-pulse/xung rác; quá cao → mất xung hoàn toàn |

> Xem giải thích chi tiết + sơ đồ dạng sóng cho từng trimpot ở phần
> **A3/A4/A5** bên dưới.

### Cài Đặt DSO152

> DSO152 chỉ có 3 chế độ trigger: **Auto** (tự động quét liên tục, hiển thị
> cả khi chưa đủ điều kiện trigger — dùng cho hầu hết các bước quan sát
> tín hiệu lặp lại/DC), **Normal** (chỉ vẽ lại màn hình khi tín hiệu thật
> sự cắt qua mức trigger — dùng khi muốn khóa hình ổn định hơn cho xung
> lặp đều), **Single** (bắt 1 lần rồi dừng — ít dùng trong quy trình này).
> Không có tùy chọn chọn cạnh lên/xuống (rising/falling) riêng.

| Bước | Probe tại | Coupling | Volt/div | Time/div | Mode |
|------|-----------|----------|----------|----------|------|
| A1a | TP_5V (U16) | DC | 2V | 10ms | Auto |
| A1b | TP_VREF (U14) | DC | 1V | 10ms | Auto |
| A2 | TP_INA_OUT (U3) | AC | 500mV | 2ms | Auto |
| A3/A4 | TP_LMV358_OUT (U1) | DC | 1V | 2ms | Auto |
| A5 | TP_RPM_OUT (U2) | DC | 2V | 2ms | Auto (hoặc Normal nếu muốn khóa hình ổn định hơn) |
| A-cuối | GPIO33 | DC | 2V | 2ms | Auto |

> GND probe **luôn cắm vào U15 (TP_GND)**.

> **Time/div theo tốc độ quay**: các giá trị Time/div ở trên (2ms) hợp khi
> rotor quay **nhanh** (quạt/khí nén mạnh, hoặc starter ở A5). Nếu **quay tay
> chậm ~1–3 vòng/giây**, một chu kỳ kéo dài ~200–500ms nên 2ms/div sẽ không
> thấy hết sóng — hãy **nới Time/div lên ~50–100ms/div** để thấy trọn 1–2 chu
> kỳ trước, rồi thu lại khi cần xem chi tiết đỉnh sóng. Volt/div và điểm nối
> probe giữ nguyên như bảng.

### GIAI ĐOẠN A — Hiệu Chỉnh Analog Bằng Oscilloscope

> **Quy ước nối DSO152 (áp dụng cho mọi bước A1–A5):**
> - **Que mát** (kẹp cá sấu, đầu ⏚) → luôn cắm cố định vào **U15 (TP_GND)**,
>   không tháo ra suốt Giai đoạn A.
> - **Que tín hiệu** (đầu nhọn) → chạm vào testpoint của từng bước; mỗi bước
>   dưới đây có dòng **🔌 Nối probe** ghi rõ testpoint và vị trí trên mạch.
> - DSO152 chỉ có **1 kênh** → chỉ đo 1 điểm mỗi lần; di chuyển que tín hiệu
>   lần lượt qua các testpoint theo thứ tự A1 → A2 → A3 → A4 → A5.

> **Bố trí sensor–magnet trên bản dựng này (đọc kỹ trước bước A2):**
> nam châm neodymium gắn **cố định** trong bánh nén/turbine bằng **nhôm**,
> cách mặt KMZ10A khoảng **~5cm** — khoảng cách do kết cấu cơ khí quyết định,
> **không chỉnh được** khi test. Nhôm không chắn từ trường nên tại ~5cm vẫn
> thu đủ biên độ (đã kiểm chứng thực tế trên bản dựng này). Do đó khi tín hiệu
> yếu/không có, **đừng đi tìm cách "đưa magnet lại gần"** — hãy soát **hướng
> đặt KMZ10A**, từ tính nam châm, và dây/nguồn.

**Cách quay magnet khi rotor nằm bên trong (không quay tay được)**

Magnet gắn trên spinner nut/compressor wheel thường nằm sâu trong cửa hút,
không phải lúc nào cũng thò tay quay trực tiếp được. Dùng phương pháp khác
nhau tùy mục đích từng bước:

- **Các bước A2-A4 (cần quan sát tín hiệu "sạch", không nhiễu điện)**:
  dùng **quạt hoặc khí nén thổi vào cửa hút** để cánh quạt/compressor wheel
  tự do quay theo luồng khí. Đây là cách quay **không sinh nhiễu điện**
  (hoàn toàn thụ động, không motor/ESC nào chạy), phù hợp nhất để chỉnh
  trimpot chính xác. Tốc độ không cần chính xác — chỉ cần quay đều vài
  giây để quan sát dạng sóng ổn định.
- **Bước A5, phần test EMI (cố ý đưa nhiễu thật vào)**: dùng **starter
  motor** qua `starttest <us> <ms>` hoặc Test Wizard — mục đích ở đây khác
  hẳn: cần nguồn nhiễu ESC/motor thật để kiểm tra RP3 đã chỉnh có đủ margin
  chống nhiễu hay không.
- Nếu rotor vẫn thò tay quay được (một số turbine cửa hút đủ rộng) — quay
  tay là cách đơn giản nhất, vẫn dùng bình thường cho mọi bước A2-A5.

**A1 — Kiểm tra nguồn** (không cần quay magnet)

> 🔌 **Nối probe**: que mát → **U15 (TP_GND)**. Que tín hiệu: bước **A1a**
> chạm **U16 (TP_5V)** — testpoint rail +5V; bước **A1b** chuyển sang
> **U14 (TP_VREF)** — testpoint mid-rail 2.5V.

- U16: DC phẳng +5.0V ±0.2V. U14: DC ổn định 2.4-2.6V.
- Sai → kiểm tra BEC/hàn R40/R39/C23/C1 trước khi tiếp tục.

**A2 — Xem tín hiệu INA826 (U3)**

> 🔌 **Nối probe**: que tín hiệu → **U3 (TP_INA_OUT)** — testpoint ngay sau
> IC khuếch đại đo lường **INA826 (U20)**; que mát → **U15 (TP_GND)**. AC coupling.

- Quay magnet ~1-3 vòng/giây (quạt/khí nén hoặc tay, xem trên), AC coupling.
- Kỳ vọng: sóng sin/quasi-sin, biên độ 200mV-1Vpp.
- Không thấy gì → khoảng cách magnet cố định (~5cm, xem ghi chú đầu Giai đoạn A)
  **không phải chỗ để chỉnh**: kiểm tra **hướng đặt KMZ10A** (xoay 90° thử),
  xác nhận nam châm còn từ tính, soát lại dây/nguồn (A1).

*Nếu sóng không ra hình sin/quasi-sin*:

| Dạng sóng thấy được | Nguyên nhân khả dĩ | Xử lý |
|---|---|---|
| Đỉnh bị cắt phẳng (clipping/vuông hóa) ngay tại U3 | INA826 bão hòa do nam châm quá mạnh (gain ×38 cố định) — hiếm gặp ở ~5cm | Khoảng cách cố định không đổi được → nếu thật sự clipping tại U3 thì thay nam châm yếu hơn. Nếu U3 vẫn tròn mà chỉ méo ở U1 → đó là clipping tầng sau, giảm RP2 (A4) |
| Răng cưa/tam giác thay vì sin | Quay không đều (quạt thổi giật cục, hoặc tay quay không đều) | Quay đều hơn — không phải lỗi mạch |
| Nhiễu loạn, không thấy chu kỳ rõ | EMI/nhiễu nền, dây tín hiệu gần dây công suất, tiếp xúc lỏng | Kiểm tra dây, tách xa nguồn nhiễu, xem lại A1 |
| "Nhọn"/không tròn nhưng vẫn đều theo chu kỳ | Bình thường với AMR sensor — đây là lý do gọi "quasi-sin" | Không cần sửa, miễn biên độ đạt 200mV-1Vpp và có chu kỳ rõ |
| Hoàn toàn phẳng, không AC | Sensor chết/sai hướng/mất nguồn | Xem lại A1, kiểm tra hướng đặt KMZ10A, xác nhận nam châm còn từ tính |

Nguyên tắc chung: miễn sóng **có chu kỳ rõ ràng, biên độ đủ, lặp lại đều
theo tốc độ quay** thì không bắt buộc phải là sin hoàn hảo — bước RP1/RP2/
RP3 tiếp theo sẽ xử lý phần còn lại.

**A3 — Chỉnh RP1 (Offset) tại U1**

> 🔌 **Nối probe**: que tín hiệu → **U1 (TP_LMV358_OUT)** — testpoint sau
> 2 tầng op-amp **LM358 (U18)**, ngay trước comparator; que mát →
> **U15 (TP_GND)**. DC coupling.

*RP1 chỉnh gì*: RP1 là trimpot hồi tiếp của tầng LM358 Op-Amp 1 — nó
**dịch mức DC (offset)** của tín hiệu analog lên/xuống, không ảnh hưởng
biên độ AC. Vặn RP1 tương đương "kéo" cả sóng lên hoặc xuống theo trục
điện áp, giữ nguyên hình dạng sóng.

*Vì sao quan trọng*: Comparator (LMV393, bước A5) so sánh tín hiệu này với
ngưỡng VTH_RPM cố định quanh 2.5V. Nếu sóng bị lệch tâm (không đối xứng
quanh 2.5V), comparator sẽ kích **không đều giữa 2 cực nam châm** (N và S)
— ví dụ cực N tạo xung tốt nhưng cực S không đủ vượt ngưỡng → **mất một
nửa số xung mỗi vòng quay**, RPM đọc được chỉ bằng nửa thực tế.

*Thao tác*: DC coupling tại U1, quay magnet đều ~2 vòng/giây, vặn RP1 cho
đỉnh dương và đỉnh âm của sóng **cách đều 2.5V**.

```
RP1 lệch (offset sai):              RP1 đúng (đã chỉnh):
    4.0V ┐ đỉnh dương                   3.5V ┐ đỉnh dương
         │  (cách 2.5V: 1.5V)                │  (cách 2.5V: 1.0V)
    2.5V ─┼─── VREF                     2.5V ─┼─── VREF (đúng tâm)
         │                                     │
    2.0V ┘ đáy                            1.5V ┘ đáy
         (cách 2.5V: 0.5V — LỆCH!)             (cách 2.5V: 1.0V — CÂN)
```

*Kiểm tra*: ghi điện áp đỉnh+ và đỉnh−, tính hiệu với 2.5V — hai số này
phải **bằng nhau** (sai lệch ≤0.1V là chấp nhận được).

**A4 — Chỉnh RP2 (Gain) tại U1**

> 🔌 **Nối probe**: **giữ nguyên** que tín hiệu ở **U1 (TP_LMV358_OUT)** như
> A3 (que mát vẫn ở U15) — A4 đo cùng điểm với A3, chỉ khác trimpot cần vặn.

*RP2 chỉnh gì*: RP2 là trimpot hồi tiếp của tầng LM358 Op-Amp 2 — nó
**thay đổi hệ số khuếch đại (gain)** của tín hiệu, tức là kéo giãn/co lại
biên độ dao động AC, không dịch tâm DC (tâm vẫn giữ ở 2.5V nhờ RP1 đã
chỉnh ở A3).

*Vì sao quan trọng*: Biên độ càng lớn thì tín hiệu càng dễ vượt qua ngưỡng
comparator (bước A5) một cách dứt khoát, có margin chống nhiễu tốt. Nhưng
nếu khuếch đại quá mức, đỉnh sóng sẽ bị **cắt phẳng (clipping)** do vượt
quá dải điện áp ra của op-amp — sóng vuông hóa một phần thay vì sin trơn,
gây ra **double-pulse** (comparator kích 2 lần trên 1 đỉnh bị méo) hoặc
đọc RPM cao giả (OVERSPEED oan).

*Thao tác*: vẫn probe tại U1, quay magnet đều, vặn RP2 để đạt biên độ
**1–2Vpp**:

```
RP2 quá thấp (<0.5Vpp):       RP2 tối ưu (1-2Vpp):        RP2 quá cao (clipping):
   2.7V ┐ đỉnh thấp              3.5V ┐ đỉnh tròn              4.5V ┌──┐ đỉnh PHẲNG
        │ không đủ vượt               │                              │  │ (op-amp bão hòa)
   2.5V ─┼─                     2.5V ─┼───                     2.5V ─┼───
        │ ngưỡng comparator           │                              │  │
   2.3V ┘ → KHÔNG ra xung        1.5V ┘ đáy tròn               0.5V └──┘ đáy PHẲNG
   (NO_SIGNAL dù đang quay)      (kích comparator rõ ràng)     (double-pulse, RPM giả cao)
```

*Nhận biết clipping*: đỉnh/đáy sóng "phẳng" thay vì bo tròn tự nhiên →
**giảm RP2**. Nếu biên độ vẫn dưới 0.5Vpp dù đã vặn RP2 tối đa → kiểm tra
lại A2 (biên độ INA_OUT quá nhỏ, thường do nam châm yếu hoặc lệch hướng
sensor — không phải do khoảng cách vì khoảng cách cố định) trước khi tiếp
tục chỉnh RP2.

> ⚠️ **Trần output thực tế của LM358 KHÔNG gần 5V**: IC lắp trên board là
> **LM358** (không phải LMV358 rail-to-rail như tên net `TP_LMV358_OUT` dễ
> gây nhầm — xem ghi chú ở đầu tài liệu). Tầng ra LM358 là kiểu cũ
> (class-AB), **không kéo lên sát V+** được — với VCC≈5V, mức cao thực tế
> chỉ đạt khoảng **3.1–3.8V** dù RP2 đã vặn hết cỡ. Đây là **giới hạn vật lý
> của chip**, không phải dấu hiệu chỉnh RP2 sai. Nếu đỉnh sóng dừng ở
> ~3.1–3.8V nhưng vẫn **bo tròn tự nhiên** (không phẳng cứng) và comparator
> (bước A5) vẫn ra xung sạch → **không cần cố vặn RP2 để kéo cao hơn**, vô
> ích và chỉ tăng nguy cơ clipping/double-pulse. Ví dụ "4.5V" trong sơ đồ
> trên chỉ minh họa khái niệm clipping nói chung, không phải mức điện áp
> literal đo được trên board này.

**A5 — Chỉnh RP3 (Threshold) tại U2, xác nhận xung RPM_OUT**

> 🔌 **Nối probe**: que tín hiệu → **U2 (TP_RPM_OUT)** — testpoint xung ra
> của comparator **LMV393 (U19)**, đây là **điểm đo quan trọng nhất**; que
> mát → **U15 (TP_GND)**. DC coupling, 2V/div. (Khi cần đọc ngưỡng bằng số
> ở cuối bước, chuyển tạm que tín hiệu sang **U21 / TP_VTH_RPM**.)

*RP3 chỉnh gì*: RP3 đặt điện áp **VTH_RPM** — ngưỡng so sánh (IN− của
LMV393 comparator). Đây là "lằn ranh" điện áp: sóng analog vượt lên trên
VTH_RPM → RPM_OUT chuyển mức cao; xuống dưới → chuyển mức thấp. RP3 không
ảnh hưởng dạng sóng analog (đó là việc của RP1/RP2) — nó chỉ quyết định
**tại điểm nào trên sóng thì được tính là 1 xung**.

*Vì sao quan trọng*: Đây là bước quyết định trực tiếp chất lượng xung
digital cuối cùng vào GPIO33 — mọi sai số ở RP1/RP2 trước đó cuối cùng
đều thể hiện ra ở đây. R36 (470K) tạo **hysteresis** (độ trễ ngưỡng) cho
comparator — nghĩa là ngưỡng bật và ngưỡng tắt lệch nhau một chút, giúp
chống nhiễu quanh điểm ngưỡng. RP3 dịch chuyển cả cặp ngưỡng bật/tắt này
lên xuống.

*Thao tác*: probe tại U2 (TP_RPM_OUT), DC coupling, 2V/div, mode Auto
(hoặc Normal nếu muốn khóa hình ổn định hơn), quay magnet đều ~2 vòng/giây.

```
RP3 quá THẤP (ngưỡng dưới VREF quá xa):
  ┌──┐  ┌─┐┌┐  ← double-pulse/xung rác: sóng dao động nhỏ quanh
  │  │  │ ││ │     ngưỡng thấp cũng đủ cắt qua nhiều lần
                → TĂNG RP3 (vặn theo chiều kim đồng hồ)

RP3 ĐÚNG (1 xung sạch/vòng):
  ┌──┐     ┌──┐
  │  │     │  │  ← đẹp, đều, cách đều nhau — đúng 1 xung mỗi cực nam châm
  ┘  └─────┘  └──

RP3 quá CAO (ngưỡng vượt quá đỉnh sóng thật):
  (không thấy xung nào dù magnet đang quay — sóng analog không bao giờ
   chạm tới ngưỡng)
       → GIẢM RP3 (vặn ngược chiều kim đồng hồ)
```

*Cách tìm điểm làm việc tối ưu (margin)*:
1. Từ trạng thái đang có xung sạch, tiếp tục **tăng RP3** từ từ cho đến
   khi xung **vừa biến mất hoàn toàn** → đây là biên trên
2. **Lùi lại 1/4 vòng** (giảm RP3) từ biên trên đó → đây là điểm làm việc
3. Margin 1/4 vòng này là vùng đệm bảo vệ khỏi **trôi ngưỡng do nhiệt độ**
   (linh kiện analog thay đổi nhẹ theo nhiệt độ động cơ) và dao động nguồn
   5V — nếu chỉnh sát biên, chỉ cần trôi nhẹ là mất tín hiệu giữa lúc đang
   chạy động cơ thật

*Test nhiễu EMI*: vẫn probe tại U2, bật starter ESC ở mức thấp (~20%),
quan sát RPM_OUT có xuất hiện spike lạ không (do EMI từ ESC/dây công suất
lan sang dây RPM). Có spike → tăng RP3 thêm ~1/8 vòng, hoặc tăng
`rpmfilter` trong firmware (xem Giai đoạn 2B).

*Đọc VTH_RPM bằng số*: chuyển probe tạm sang U21 (TP_VTH_RPM), ghi lại
điện áp ngưỡng tại điểm làm việc tối ưu — con số này dùng để đối chiếu
nếu sau này nghi ngờ trimpot bị trôi/lệch.

**A-cuối — Xác nhận GPIO33**: cắm J_RPM_OUT1→RPM_PIN_IN1.

> 🔌 **Nối probe**: que tín hiệu → chân **GPIO33** trên ESP32 (hoặc đầu
> **RPM_PIN_IN1** trên board ECU, sau R2/D1); que mát → **U15 (TP_GND)**
> hoặc chân **GND của ESP32** (đã chung mát với mạch RPM). DC coupling, 2V/div.

Kỳ vọng xung **0-3.3V** (D1 TVS clamp giữ không vượt 3.3V). Nếu vẫn thấy 5V
→ D1 lỗi, thay ngay trước khi cấp nguồn ESP32.

### GIAI ĐOẠN B — Quét Toàn Dải PWM Bằng Firmware TEST_STARTER

Dùng `TEST/TEST_STARTER/TEST_STARTER.ino` (firmware bench riêng, không
cần ARM, không bảo vệ) để quét **toàn dải PWM starter thật**, thay vì chỉ
quan sát bằng tay quay chậm trên oscilloscope.

**Đấu nối**: Starter ESC signal → GPIO25, RPM sensor → GPIO33 (giống mạch chính).

**⚠️ An toàn**: THÁO cánh/impeller khỏi starter trước khi test. Nguồn ESC
tách riêng nguồn ESP32, đủ dòng (xem phần Nguồn & Pin).

**Upload**: mở `TEST/TEST_STARTER/TEST_STARTER.ino`, board ESP32 Dev Module,
không cần lib ngoài (LEDC PWM có sẵn trong core).

**Test qua Serial HOẶC qua Web** (không bắt buộc phải dùng Serial Monitor):
- Serial: 115200 baud, Newline
- Web: SoftAP **TEST_STARTER** / pass **test1234** → `http://192.168.4.1` —
  cùng bộ lệnh qua nút bấm/ô nhập, tiện khi test bằng điện thoại

**Bảng lệnh**:

| Lệnh | Chức năng |
|------|-----------|
| `+` / `-` | Tăng/giảm PWM starter 1 bước (mặc định 10µs) |
| `0` hoặc `s` | Dừng starter (PWM về 1000µs) |
| `step <us>` | Đổi bước tăng/giảm (1..200) |
| `pwm <us>` | Đặt PWM trực tiếp (1000..2000) |
| `ppr <n>` | Số xung/vòng (1=nam châm, 2=quang học) |
| `filter <us>` | Bộ lọc glitch RPM (20..2000), mặc định 120 |
| `edge rising\|falling` | Cạnh kích RPM |
| `reset` | Xóa bộ đếm & lịch sử RPM |
| `status` / `help` | In trạng thái / menu lệnh |

> Firmware ECU chính có mục **"Starter Manual Test"** trên Web UI dashboard
> cho phép chạy `starttest <us> <ms>` trực tiếp từ trình duyệt (không cần
> arm2) — không bắt buộc phải dùng Serial.

**Đọc dòng trạng thái** (in 2 lần/giây):
```
PWM=1200us | RPM=8450 (win 8410) | raw=141 acc=141 rej=0 (0.0%) | jit=4.2% cv=1.1% | NOISE=CLEAN STAB=STABLE
```

| Trường | Ý nghĩa |
|--------|---------|
| `PWM` | Mức PWM hiện tại. `(OFF)` khi =1000µs |
| `RPM` / `win` | RPM tức thời / trung bình cửa sổ 100ms |
| `raw`/`acc`/`rej` | Cạnh thô / chấp nhận / bị lọc |
| `jit` | Jitter chu kỳ (max-min)/avg |
| `cv` | Hệ số biến thiên RPM (stddev/mean) |
| `NOISE`/`STAB` | Đánh giá nhiễu / độ ổn định tự động |

**Ngưỡng NOISE**: `CLEAN` (reject≤5%, jit≤15%) · `WARN` (reject>5% hoặc jit>15%)
· `NOISY` (reject>20% hoặc jit>40%) · `NO_SIGNAL` (không cạnh nào).

**Ngưỡng STAB** (sau khi PWM ổn định 1.5s): `STABLE` (CV≤2%) · `WARN` (2-5%)
· `UNSTABLE` (>5%) · `SETTLING` (mới đổi PWM) · `OFF` (PWM=1000µs).

**Quy trình quét**:
1. Test tĩnh (PWM=OFF) 30s: kỳ vọng `raw≈0`, `NOISE=NO_SIGNAL`. Có `acc>0` → nhiễu nền, quay lại A5.
2. Rà quét `+` từng bước nhỏ (vd +10µs từ 1100µs), chờ `STAB` khỏi `SETTLING` rồi ghi `NOISE`/`STAB`.
3. Ghi bảng PWM→RPM→NOISE→STAB (mẫu bên dưới).
4. `NOISY` ở PWM cao → tăng `filter 300`, thêm tụ 100nF tại GPIO33, tách dây RPM xa dây ESC. Vẫn không hết → quay lại A5 tăng margin RP3.
5. **GO/NO-GO**: toàn dải phải đạt `NOISE=CLEAN` và `STAB=STABLE` trước khi chuyển sang firmware ECU chính.

**Bảng ghi kết quả quét** (điền khi test):

| PWM (µs) | RPM | raw | acc | rej% | jit% | cv% | NOISE | STAB |
|----------|-----|-----|-----|------|------|-----|-------|------|
| 1000 (OFF) | | | | | | | | |
| 1100 | | | | | | | | |
| 1150 | | | | | | | | |
| 1200 | | | | | | | | |
| 1250 | | | | | | | | |

**Xác nhận cuối** (trên firmware ECU chính, không phải TEST_STARTER):
```
rpmdetail on
```
Quay tay đều ~1 vòng/giây → Serial phải hiện RPM ≈ 60 (ppr=1), `NOISE=CLEAN`,
`rej%` < 5%.

---

## Giai đoạn 3 — Test từng bộ phận (Test Wizard, KHÔNG nhiên liệu)

Các lệnh test **không cần `arm2`** (đã bỏ yêu cầu ARM cho phần test, chỉ còn
yêu cầu ở `startidle`/`autostart on`/tắt interlock — xem Giai đoạn 5).

```
test ign          -> glow plug bật 1s, quan sát dòng điện/nhiệt bằng tay (cẩn thận nóng)
test starter      -> starter quay 5s ở us cấu hình, xem RPM có tăng lên không
                     (RPM/RNOISE được chốt LÚC starter còn quay, ngay trước khi tắt —
                      tránh rest-guard ép RPM=0; 5s đủ để tốc độ ổn định, hết báo NOISY)
test starter_ign  -> starter + glow cùng lúc, kiểm tra EMI có làm nhiễu RPM không (resetRpmStats() trước bước này đã fix trong CODE_REVIEW_FINDINGS.md)
test valve1        -> Valve 1 = Start solenoid valve
test valve2        -> Valve 2 = Main oil valve
```

Với mỗi test, dùng `checklist` để xem kết quả — tất cả phải chuyển từ
`NOT_RUN` → `PASS`. Nếu `FAIL` — sửa phần cứng trước khi tiếp tục (đừng
chỉnh firmware để né test).

> **Chặn khi EGT còn nóng**: `test ign`/`test starter_ign`/`test valve1`/`test valve2`
> đều bị chặn nếu EGT còn nóng hơn `cooldownTargetC` (giống hệt lệnh trực tiếp
> `ignpulse`/`valve on`) — tránh châm lửa/mở van vào lõi còn nóng sau abort.
> Riêng `test starter` (chỉ starter) vẫn cho chạy vì giúp làm mát. Nếu test bị
> `BLOCKED` do nóng, đợi nguội hoặc chạy tiếp cooldown trước khi test lại.

**Component test khác (tùy chọn, bench-only, không cần arm2)**:
```
starttest <us> <ms>   -> vd: starttest 1200 3000
ignpulse <ms>          -> vd: ignpulse 1500
valve1 on/off | valve2 on/off
```

> ⚠️ **`valve1 on`/`valve2 on` KHÔNG còn tự tắt sau 10s** — van sẽ mở cho tới
> khi bạn gõ `valve1 off`/`valve2 off` (hoặc bấm nút OFF trên Web UI). Luôn
> nhớ tắt van sau khi test xong, đặc biệt khi có nhiên liệu thật.

---

## Giai đoạn 4 — Test bơm nhiên liệu riêng (KHÔNG gắn ống vào engine)

⚠️ Tháo ống dẫn nhiên liệu ra khỏi engine, xả vào cốc hứng riêng.

```
pumptest 1160 1500   -> bơm chạy 1500ms ở 1160us, đo ml thực tế đối chiếu bảng calib
```

> Trần bench `pumptest` hiện là **1000..1225µs** (thời lượng `ms` **không giới hạn**
> — nhập bao nhiêu chạy bấy nhiêu, tự tắt sau `ms`; `starttest` cũng vậy). Nút "Pump prime" trên Web UI
> chạy ở `pumpTestUs` = **1160µs** (mặc định) — mức flow **thấp nhất đã đo thực
> tế** (50 ml/phút), an toàn hơn cho bench test. Vẫn nên quét vài mức us khác
> để xác nhận độ tuyến tính (dùng panel **"Pump Manual Test"** trên Web UI,
> hoặc lệnh `pumptest <us> <ms>` trực tiếp qua Serial).
>
> **Các mức PWM này chỉnh trực tiếp được** (không hard-code): qua Serial
> `set intro/idleus/maxus/pumptestus <us>` và `set purgeus/spinus/assistus <us>`,
> hoặc qua panel **"Starter & Fuel PWM"** trên Web UI (tự nạp giá trị từ config).
> Lưu ý: các lệnh tune PWM/limit chỉ nhận khi ở **WAITING/ABORTED**, không nhận
> khi đang chạy.

**Bảng hiệu chuẩn pump** (đo thực tế, khớp firmware):

| ESC (µs) | Lưu lượng (ml/phút) |
|----------|---------------------|
| 1000 | 0 (tắt) |
| 1160 | 50 (tối thiểu) |
| 1175 | 80 |
| 1250 | 265 |
| 1260 | 280 |
| 1265 | 360 |
| 1270 | 560 |
| 1300 | 600 (tối đa) |

Lặp lại 2-3 mức us khác nhau để xác nhận độ tuyến tính. Đây là bước cuối
để pass `TEST_PUMP` trong checklist:
```
test pump
```

Sau khi xong, `confirmkill` để xác nhận kill-switch vật lý đã test cơ học:
```
confirmkill
```

**Kiểm tra checklist tổng**:
```
checklist
```
→ `STARTIDLE_CHECK=PASS: OK` (không phải OK_DRY_START_EGT_BYPASS, vì đó là
chế độ né checklist khi cảm biến EGT hỏng — chỉ dùng khi cố ý test khô).

---

## Giai đoạn 5 — Dry-start (quay engine thật, KHÔNG nhiên liệu/lửa)

Mục đích: xác nhận starter đủ mạnh quay trục thật + RPM sensor đọc đúng khi
lắp lên engine, trước khi cho nổ.

Cách kích hoạt dry-start: rút giắc/để hở mạch cảm biến EGT tạm thời (hoặc
dùng `set egtstart dry` để cho phép), lúc đó `startidle` sẽ tự động thành
dry-run (không nhiên liệu/van/mồi lửa vì `dryStartActive=true`). **Không có
lệnh `drystart` riêng** — đây là hành vi tự động của `startidle` khi EGT lỗi.

```
set egtstart dry
arm2
autostart on
arm2
startidle
```

Quan sát: starter quay đúng `dryStartRunMs`, RPM tăng theo, không có mùi
nhiên liệu, không có tia lửa. Dùng `rpmdetail on` để xem RPM có track đúng
tốc độ quay thật không.

Xong bước này, trả cảm biến EGT về đúng dây và:
```
set egtstart strict
```
để khóa lại — từ giờ mọi lần EGT lỗi sẽ chặn start thay vì lặng lẽ dry-run.

**Dừng dry-start**: `stop` (soft stop, vào cooldown) hoặc nhấn button vật lý.

---

## Giai đoạn 6 — Wet-start (khởi động thật, có nhiên liệu + lửa)

**Chỉ làm khi**: Giai đoạn 0-5 đều PASS, EGT gắn đúng và đọc chính xác,
engine cố định chắc chắn, có người đứng cạnh kill switch, có bình chữa cháy.

```
arm2
autostart on
arm2
startidle
```

Firmware tự chạy chuỗi:
`PURGE → SPINUP_PREHEAT → INTRO_FUEL → LIGHTOFF → ACCEL_TO_IDLE → IDLING`

- **PURGE**: starter giữ **ESC_SAFE (1000µs, thật sự 0)** trong `escarmholdms`
  (**2s**) đầu để ESC arm xong trước khi nhận xung khác 0 (tránh "quay vài vòng rồi
  khựng" do ESC mới arm được một phần), rồi giữ nguyên `rampfromus` (1150µs) trong
  `purgestablems` (**10s**) cho ổn định, sau đó mới ramp **+1µs mỗi `rampstepms`
  (250ms ≈ 4µs/s)**, tới khi **RPM > `ignarmrpm` (3000)** thì giữ `purgeTimeMs` (3s)
  thổi sạch. Quá `spinuptimeoutms` (57s, đã cộng sẵn 2s arm-hold + 10s ổn định) không
  đạt RPM → abort `NO_RPM`. Áp dụng cho cả dry test lẫn chạy thật (dùng chung hàm ramp).
- **SPINUP_PREHEAT**: **glow ON `preheatMs` (2s)**, sau đó giữ glow bật.
- **INTRO_FUEL**: bơm nhiên liệu min (`introus` = 1015µs) + mở **Start Valve (Valve 1)**.
- **LIGHTOFF**: chờ `fueldelayms` (1s); xác nhận bắt lửa THẬT khi **EGT ≥ 100°C VÀ dEGT/dt ≥
  `lightoffrise` (15°C/s) giữ `lightoffconfirmms` (0.7s)** → mở **Main Valve (Valve 2)**, chờ
  `postIgnitionHeatMs` (2s), đóng Start Valve, chờ `flameprovems` (2s) xác nhận không mất lửa →
  tắt glow, thành công. Quá `noIgnitionTimeoutMs` (6s) hoặc mất lửa → thất bại (blow-out `purgeoutms` rồi cooldown).
- **ACCEL_TO_IDLE**: nếu lửa còn cháy mà RPM không tăng thì bơm +`accelstepus` mỗi `accelholdms`,
  tới `idleRpm` (42000). Starter trợ lực bám RPM (`assistus`→`ramptous`=1450µs) tới `startermaxrpm`
  (10000) rồi nhả.
- **Mất lửa**: EGT tụt > `flameoutdropc` (10°C) trong 2s **hoặc** < 100°C.

> Toàn bộ mốc trên chỉnh được trong **Web UI → Settings → "Tune — Start sequence"** và lưu vào SD.
> Xem chi tiết trong `QUY_TRINH_KHOI_DONG.md`.

### Bảo vệ động cơ / starter (chạy mọi lúc, mọi mode)

Hai cơ cấu bảo vệ chạy mỗi vòng lặp, đè lên lệnh starter của từng giai đoạn:

1. **Chống khí nóng lan lên đầu máy (soak-back)**: khi **EGT > `cooldownTargetC` (90°C)**
   mà **RPM < `hotSpinMinRpm` (3000)** → starter tự chạy, **bắt đầu ở `hotSpinUs` (1200µs,
   KHÔNG phải 1150µs như ramp lúc khởi động)** rồi **tăng dần +1µs mỗi 0.1s** cho tới khi
   RPM vượt 3000 (giới hạn trần ở `ramptous` = 1450µs). Ramp nhẹ từ 1200 tránh làm cánh
   quạt giảm tốc đột ngột khi guard vào cuộc; khi RPM đã đạt thì **giữ nguyên** mức xung
   (không tăng tiếp, không thả). Dùng cho trường hợp động cơ đang nóng mà dừng/tụt tua —
   giữ luồng khí đúng chiều, tránh khí nóng dội ngược làm hỏng đầu/ổ trước. Khi **EGT < 90°C**
   starter được phép dừng (cooldown cũng kết thúc ở mốc này) và ramp hot-spin reset về 1200µs.
2. **Chống quá tải starter**: khi **RPM > `starterMaxRpm` (15000)** → starter bị **tắt**
   (nhả khỏi cánh quạt) để rotor đã lên tua không kéo quá tải/quá tốc motor đề.

Chỉnh trong **Web UI → Settings → "Bảo vệ động cơ / starter"** (hoặc `set startermaxrpm`,
`set hotspinminrpm`, `set hotspinus`, `set cooltarget`), lưu vào SD như các tham số khác.

Theo dõi bằng Web UI (gauge RPM/EGT + chip STAGE) hoặc `status`:
- `STAGE=` chuyển đúng thứ tự trên
- `EGT=` tăng qua 100°C khi bắt lửa
- `RPM=` tăng dần đến gần idleRpm

Nếu bất kỳ đâu bị **ABORT** (OVER_TEMP, NO_IGNITION, NO_RPM_RISE,
RPM_SIGNAL_LOST, OVERSPEED, COMM_TIMEOUT...) — **đọc kỹ lý do trên Serial
trước khi `clearabort`**. Không type `clearabort` theo phản xạ.

### ⚠️ Xả nhiên liệu tồn dư trước khi thử start lại (sau NO_IGNITION / ACCEL_TO_IDLE_TIMEOUT)

Theo manual EnJet E80/E100 (`REFERENCES/Documentation/Engine_Manual_E80_E100.pdf`):
nếu lần start trước bị fail (đặc biệt `NO_IGNITION` — đã phun dầu nhưng
không bắt lửa), nhiên liệu chưa cháy **vẫn còn đọng lại trong buồng đốt**.
Cố start lại ngay có thể gây cháy lớn khi lượng dầu tồn dư này bắt lửa
đột ngột cùng lúc với lần mồi mới.

**Quy trình xả trước khi start lại**:
1. Nghiêng động cơ, **đuôi (exhaust) hướng xuống dưới**
2. Dùng `starttest <us> <ms>` (hoặc quay tay nếu có thể) để quay rotor vài
   giây, không phun thêm nhiên liệu — mục đích thổi bay dầu tồn đọng ra
   khỏi buồng đốt qua đuôi
3. Sau đó mới `clearabort` và thử `startidle` lại

Bỏ qua bước này nếu lần fail là do abort SỚM (`NO_STARTER_RPM`,
`OVER_TEMP` trước khi có nhiên liệu) — chỉ áp dụng khi đã ở stage
`ST_INTRO_FUEL` trở lên (đã phun dầu) mà chưa đánh lửa thành công.

Khi vào `MODE_IDLING` ổn định vài chục giây, không dao động RPM bất
thường → **giai đoạn wet-start thành công**.

---

## Giai đoạn 7 — Tăng throttle (chạy thật có tải)

Chỉ làm khi IDLING đã ổn định. Lệnh throttle:

```
set throttle <0..100>
```

Cơ chế: `targetRpm = idleRpm + (maxRpm - idleRpm) * throttlePct / 100`,
ECU tự động chuyển `MODE_IDLING → MODE_OPERATING` khi `throttlePct > 0`
và closed-loop điều khiển bơm để đạt RPM mục tiêu.

**Quy trình tăng dần, không nhảy vọt**:

| Bước | Lệnh | Giữ tối thiểu | Quan sát |
|------|------|--------------|---------|
| 7.1 | `set throttle 10` | 15-30s | RPM tăng nhẹ, EGT tăng từ từ, không dao động |
| 7.2 | `set throttle 25` | 15-30s | RPM ổn định ở target mới, EGT < maxEgtC |
| 7.3 | `set throttle 50` | 15-30s | Kiểm tra `rpmdetail` — jitter thấp, không NOISY |
| 7.4 | `set throttle 75` | 15-30s | EGT gradient theo dõi kỹ (dEGT) |
| 7.5 | `set throttle 100` | Chỉ khi đã quen động cơ | Full power — luôn sẵn sàng `stop` hoặc kill switch |

**Giảm ga về idle**: `set throttle 0` → tự động `MODE_OPERATING → MODE_IDLING`.

**Dừng máy an toàn (có làm mát)**: `stop` → vào `MODE_COOLDOWN`, starter
quay làm mát, tắt nhiên liệu/van, chờ `cooldownTargetC` rồi mới tắt hẳn.

**Dừng khẩn cấp**: nhấn kill switch vật lý (NC) — bypass hoàn toàn
firmware, ngắt điện trực tiếp. Lưu ý `set commwatchdog` sẽ tự abort nếu
mất kết nối Serial/Web quá `commTimeoutMs` lúc IDLING/OPERATING — xem
`CODE_REVIEW_FINDINGS.md`.

---

## Ma Trận Quyết Định GO/NO-GO (trước mỗi lần start engine thực)

| Hạng mục | GO | NO-GO |
|----------|-----|-------|
| EGT đọc nhiệt độ phòng (15-35°C) khi test tĩnh | ✅ | ❌ |
| RPM = 0 khi không quay (test tĩnh) | ✅ | ❌ |
| Giai đoạn 2B: toàn dải PWM đạt NOISE=CLEAN, STAB=STABLE | ✅ | ❌ |
| 9/9 Test Wizard PASS | ✅ | ❌ |
| Dry-start hoàn thành không lỗi | ✅ | ❌ |
| Không rò rỉ nhiên liệu, pump test khớp bảng calib | ✅ | ❌ |
| Kill switch đã confirmkill | ✅ | ❌ |
| SD card ghi log được | ✅ | ⚠️ Tùy chọn |
| Không có ABORT chưa được giải thích rõ | ✅ | ❌ |
| Nguồn/pin đủ dòng (đã đo sụt áp lúc test starter_ign) | ✅ | ❌ |

**Quy tắc GO**: tất cả ✅ trước khi nhấn `startidle` cho wet-start thật.

---

## Xử Lý Lỗi Thường Gặp

**RPM nhiễu cao (rejectPct > 20-30%)**
```
set rpmfilter 300
```
Lặp lại Giai đoạn 2B. Vẫn fail → kiểm tra wiring, thêm tụ 100nF tại GPIO33,
tách dây RPM xa dây công suất ESC. Vẫn fail → quay lại Giai đoạn 2A tăng
margin RP3.

**EGT không đọc được (FAULT/NaN)**
1. Kiểm tra thermocouple loại K, kết nối MAX31855 (CLK=18, CS=5, DO=19)
2. Đo điện áp cấp cho MAX31855
3. Thử thermocouple khác để loại trừ hỏng cảm biến

**Pump ESC không phản hồi**
1. Kiểm tra ESC arming (nghe beep khi cấp điện)
2. Đảm bảo nhận đúng tín hiệu PWM tại GPIO26
3. Đo PWM bằng oscilloscope/servo tester nếu nghi ngờ

**Starter chạy nhưng RPM không tăng**
1. Kiểm tra vị trí/hướng đặt sensor so với magnet (khoảng cách cố định ~5cm)
2. `rpmdetail on` — xem `raw` có tăng theo starter không
3. Kiểm tra nguồn 5V cho mạch RPM sensor (qua UBEC riêng)

**Starter khựng/giật giật rồi dừng dù firmware vẫn gửi PWM đều**
→ Đây là dấu hiệu **thiếu dòng cấp nguồn**, không phải lỗi firmware. Xem
phần "Nguồn & Pin" ở trên — đổi sang nguồn đủ dòng (ATX PSU, bench PSU,
hoặc pin LiPo đã tính margin phù hợp).

**Upload thất bại / không kết nối USB**
→ Xem phần "Cài Đặt Arduino IDE & Upload Firmware" ở trên.

---

## Nhật Ký Test (điền khi test thực tế)

```
Ngày test: _______________
Firmware: ECU_TestV1_EGT_DRY_START_PATCH
SD Log file: ECU___.CSV
Nhiệt độ phòng: ___°C

Giai đoạn 0 (đấu nối vật lý): PASS / FAIL
Giai đoạn 1 (test tĩnh): PASS / FAIL
Giai đoạn 2A (trimpot RP1/RP2/RP3): PASS / FAIL
  VTH_RPM tại điểm làm việc = _____ V
Giai đoạn 2B (quét PWM TEST_STARTER):
  rejectPct khi quay tay = ___%   NOISE cuối = _______
  Toàn dải CLEAN/STABLE? PASS / FAIL
Giai đoạn 3 (Test Wizard 9 bước): PASS / FAIL
Giai đoạn 4 (Pump test + confirmkill): PASS / FAIL
Giai đoạn 5 (Dry-start): PASS / FAIL
Giai đoạn 6 (Wet-start): PASS / FAIL
Giai đoạn 7 (Throttle tối đa đã test an toàn): ____%

GO / NO-GO cho lần chạy tiếp theo: _______

Ghi chú / Vấn đề gặp phải:
_______________________________________________
_______________________________________________
```

---

## Bảng Tóm Tắt Thứ Tự Toàn Bộ

```
0. Chuẩn bị an toàn + kiểm tra đấu nối vật lý
1. Test tĩnh: EGT, RPM_NOISE, SD                          → không quay gì
2. RPM sensor: 2A (trimpot DSO152) + 2B (quét PWM TEST_STARTER)
3. Test Wizard từng bộ phận (ign/starter/starter_ign/valve1/valve2)
4. Test bơm riêng (pumptest, không gắn engine) + confirmkill
5. Dry-start (quay thật, không nhiên liệu/lửa)
6. Wet-start (đánh lửa thật, có bước xả nhiên liệu tồn dư nếu fail) → IDLING ổn định
7. Tăng throttle từ 10% → 100% từng nấc, theo dõi EGT/RPM
```

---

## Lưu Ý An Toàn Xuyên Suốt

- **Không** dùng `set checklist off` khi test thật — chỉ dùng khi debug bench.
- **Không** dùng `clearabort force` trừ khi chắc chắn máy đã nguội và cảm
  biến EGT hỏng vĩnh viễn — lệnh này bỏ qua xác nhận nhiệt độ qua sensor.
- Sau **mọi ABORT**, đọc lý do (`lastAbortReason`) trước khi re-arm, đừng
  `clearabort` phản xạ.
- Sau `NO_IGNITION`/abort có phun dầu nhưng chưa cháy — xả nhiên liệu tồn
  dư (nghiêng đuôi xuống + quay rotor không tải) trước khi thử lại, xem
  Giai đoạn 6.
- Không gõ lệnh Serial chậm/ngắt quãng khi engine đang chạy thật.
- Không để SD đầy 1000 file log.
- Throttle luôn tăng/giảm từng nấc nhỏ, không set thẳng 100% từ IDLING.
- Nguồn/pin phải đủ dòng xả — nguồn yếu gây triệu chứng giống lỗi cơ khí
  (starter khựng) nhưng thực chất là thiếu dòng, xem phần "Nguồn & Pin".

---

**Tài liệu liên quan**:
- `PROJECT_CURRENT/Docs/CODE_ARCHITECTURE.md` — hiểu sâu kiến trúc firmware
- `PROJECT_CURRENT/Docs/CODE_REVIEW_FINDINGS.md` — các lỗi đã fix qua các vòng review, ảnh hưởng độ tin cậy các bước trên
- `REFERENCES/Documentation/Engine_Manual_E80_E100.pdf` — nguồn quy trình xả nhiên liệu tồn dư (Giai đoạn 6)
- `REFERENCES/Documentation/Engine_Manual_NewerModel.pdf` / `_Chinese.pdf` — manual EnJet G3/E86, nguồn valve1/valve2 role split
- `TEST/TEST_STARTER/TEST_STARTER.ino` — firmware bench dùng ở Giai đoạn 2B

**Phiên bản**: 3.1 — 2026-07-18 (gộp Upload/Test + Pre-Engine-Test + DSO152 + Commissioning thành 1 file)
