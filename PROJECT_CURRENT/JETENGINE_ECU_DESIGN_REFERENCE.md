# Tài Liệu Tham Khảo Thiết Kế ECU Động Cơ Phản Lực (Enjet E86)

## Tổng Quan Dự Án

Tự chế hệ thống ECU (Engine Control Unit) cho động cơ phản lực mini **Enjet E86** — một thử thách lớn về cả phần cứng lẫn lập trình phần mềm. Động cơ này quay tối đa **152.000 RPM** và nhiệt độ buồng đốt có thể vượt quá **700°C**.

---

## 1. Khối Điều Khiển Motor Đề (Starter)

### 1.1 Vấn Đề Khởi Động Hiện Tại

Mạch ECU hiện tại gặp vấn đề khi khởi động:
- **Lỗi reset** xảy ra ở mức xung **~1465µs** (motor tải nặng từ cánh nén)
- **Nguyên nhân**: ESC BLHeli_S 30A được thiết kế tối ưu cho Drone (phản xạ nhanh, thay đổi tốc độ tức thì), nhưng khi tải là cánh nén (compressor) tải nặng tăng dần theo tốc độ → sốc dòng, sụt áp pin, reset vi điều khiển

### 1.2 Cải Thiện ESC Firmware

Cần nạp lại firmware BLHeli_S trên ESC đề bằng mạch CH340 + cáp 3 dây (GND, TXD, RXD):

| Tham số | Giá trị | Lý do |
|---------|---------|------|
| **Startup Power (Công suất khởi động)** | 0.125 – 0.25 (rất thấp) | Motor khởi động từ từ, không bị sốc dòng |
| **Motor Timing** | High | Giữ từ trường tối ưu khi tốc độ thấp |
| **Acceleration** | Slope Medium–High | Tăng ga nhẹ nhàng thay vì bốc |
| **Brake on Stop** | Enabled | Giữ motor dừng lại, tránh quay ngoài ý muốn |

### 1.3 Thuật Toán Tăng Xung PWM (Ramping Log) Trong Firmware ECU

**Bắt buộc**: Không được cho xung PWM nhảy thẳng từ 1000µs lên 1450µs — phải tăng dần từng bước nhỏ:

```
Ví dụ: Tăng mượt từ 1000µs → 1450µs
  Mỗi 10 ms: +1µs
  → Thời gian tổng: (1450 - 1000) × 10 = 4500 ms = 4.5 giây
  
Hoặc thậm chí chậm hơn nữa:
  Mỗi 20 ms: +1µs → 9 giây để lên tới 1450µs
```

**Lợi ích**:
- Motor có thời gian để **thắng lực cản cánh nén khí**
- Dòng điện tăng dần → không sốc, không sụt áp
- Pin cấp nguồn ổn định, vi điều khiển không reset

**Pseudocode**:
```cpp
void startMotor() {
  unsigned long lastStep = millis();
  int targetUs = 1450;
  int currentUs = 1000;
  
  while (currentUs < targetUs) {
    unsigned long now = millis();
    if (now - lastStep >= 10) {  // Mỗi 10ms tăng 1us
      currentUs++;
      escWriteUs(currentUs);
      lastStep = now;
    }
  }
}
```

---

## 2. Các Khối Cảm Biến Bắt Buộc Trên Mạch ECU

### 2.1 Khối Đo Tốc Độ Vòng Quay (RPM Sensor)

**Phần cứng**:
- Loại cảm biến: **Cảm biến từ trường (Hall effect)** hoặc **cảm biến quang học (Infrared)**
- Vị trí đặt: **Mũi động cơ** (phía trước cánh nén)
- Phương pháp: Đếm số vòng quay (nam châm/vạch sơn trên trục)
- **Mạch xử lý hiện tại**: KMZ10A (AMR) + INA826 (khuếch đại) + LMV358 (offset/gain) + LMV393 (comparator) → GPIO33

**Nhiệm vụ trong ECU**:
1. **Đo RPM liên tục** = cơ sở feedback cho các quyết định điều khiển
2. **Điểm chuyển tiếp động cơ**:
   - Khi RPM đạt **~30.000 RPM** → buồng đốt đã bắt lửa tự duy trì
   - **Ngắt hoàn toàn xung cấp cho motor đề** (ESC nhận 1000µs = 0% throttle)
   - Kích hoạt **ly hợp một chiều** tự nhả (freewheel) → động cơ tự vận hành độc lập
3. **Giám sát an toàn**:
   - Nếu RPM không tăng sau 5–10 giây khởi động → **fail to ignite** → tắt toàn bộ

### 2.2 Khối Đo Nhiệt Độ Buồng Đốt (EGT - Exhaust Gas Temperature)

**Phần cứng**:
- Loại cảm biến: **Nhiệt điện cặp loại K (Thermocouple Type K)**
- Phạm vi: 0–1300°C
- IC khuếch đại: **MAX6675** hoặc **MAX31855** (output SPI/I2C)
- Vị trí: Ống xả động cơ, gần buồng đốt (vị trí có nhiệt độ cao nhất)

**Nhiệm vụ trong ECU**:
1. **Xác nhận bắt lửa thành công**:
   - Khi mở van nhiên liệu mồi (Bước 3), cảm biến EGT phải báo nhiệt độ tăng lên (>150°C) trong 2–3 giây
   - Nếu không → **tắt toàn bộ**, báo lỗi "Fail to Ignite"
2. **Giám sát Over-Temperature**:
   - Nếu EGT vượt quá **850°C** → **tắt bơm dầu ngay**, giảm tốc độ động cơ
   - Nếu vượt quá **900°C** → **shutdown khẩn cấp** để tránh hủy động cơ
3. **Tối ưu hiệu suất**:
   - Giữ EGT ở **vùng tối ưu 750–820°C** bằng cách điều chỉnh tốc độ bơm dầu

---

## 3. Khối Điều Khiển Bơm Nhiên Liệu (Fuel Pump) và Van Solenoid

### 3.1 Bơm Nhiên Liệu (Fuel Pump)

**Thông số**:
- Loại: Motor brushed (chổi than) nhỏ + hộp số giảm tốc
- Nhiên liệu: Kerosene, Diesel, hoặc Jet-A1
- Điều khiển: **Xung PWM** (tốc độ bơm ∝ duty cycle PWM)

**Mạch điều khiển trên ECU**:
- **Cầu H (H-bridge)** hoặc **Mosfet** chịu dòng cao (≥ 30A)
- **GPIO từ ESP32** → Mosfet → Motor bơm
- Đo dòng bơm: Có cảm biến dòng (Current Sensor) tùy chọn để báo hiệu tắc dầu

**Thuật toán cấp dầu (Fuel Mapping)**:
```
Tốc độ bơm dầu (%) = f(RPM)
  
Vùng tua thấp (0 – 10.000 RPM):
  Bơm dầu rất ít (0–5%), tránh bị nghẹn
  
Vùng tua trung (10.000 – 30.000 RPM):
  Tăng tuyến tính theo RPM (5% → 50%)
  
Vùng tua cao (30.000 – 152.000 RPM):
  Bơm dầu cao (50% – 100%), tối ưu hoàn toàn
  
⚠️ Lưu ý: Nếu bơm quá nhiều khi tua còn thấp → động cơ bị nghẹn, lửa phụt
```

### 3.2 Cảm Biến Lưu Lượng Nhiên Liệu (Fuel Flow Sensor) — Closed-Loop Feedback

**Phần cứng**:
- Loại cảm biến: **ZJTM-04A-1.2** — turbine flow sensor dùng Hall effect (nguyên lý giống KMZ10A RPM: bánh turbine quay theo dòng chảy, nam châm trên turbine tạo xung mỗi vòng qua cảm biến Hall)
- Điện áp hoạt động: **3.5–24VDC**
- Ngõ ra: xung digital (tần số ∝ lưu lượng)
- Vị trí lắp: **ngay sau bơm nhiên liệu, trước van solenoid dầu chính** — đo lưu lượng thực tế đã bơm ra, không phải suy đoán từ duty cycle PWM

**Công thức hiệu chuẩn**:
```
F = 190 × Q − 5        (F: tần số Hz, Q: lưu lượng L/phút)

Suy ra dùng cho firmware:
Q = (F + 5) / 190

Số hạng −5: ngưỡng chết (dead-zone offset) — turbine cần lưu lượng tối
thiểu để thắng ma sát vòng bi trước khi bắt đầu quay, nên ở lưu lượng
rất thấp tần số đo được lệch khỏi quan hệ tuyến tính lý thuyết.
```

**Sơ đồ điện** — tái sử dụng đúng pattern bảo vệ GPIO đã dùng cho RPM_OUT (R10 pull-up 10K + R2 series 1K + D1 TVS clamp 3.3V):

```
UBEC 5V ──┬──→ ESP32 VCC
          └──→ ZJTM-04A-1.2 VCC (3.5-24V, dùng 5V phù hợp)
                    │
                   GND ──────────→ GND chung
                    │
                   OUT (xung) ──┬──→ R_pullup (10K) → VCC sensor
                                │
                                R2 (1K series)
                                │
                                ├──→ D1 (TVS 3.3V clamp) → GND
                                │
                                ↓
                           GPIO_FLOW (ESP32, chọn chân input-only
                           rảnh: GPIO34/35/36/39)
```

⚠️ **Không cấp nguồn sensor từ 3.3V ESP32** — dưới ngưỡng tối thiểu 3.5V, sensor sẽ không hoạt động ổn định.

**Firmware — đo tần số xung** (kiến trúc giống hệt code đếm RPM đã có, chỉ khác hằng số hiệu chuẩn):
```cpp
volatile unsigned long flowPulseCount = 0;

void IRAM_ATTR flowISR() {
  flowPulseCount++;
}

// Mỗi 200ms trong loop:
unsigned long count = flowPulseCount;
flowPulseCount = 0;
float freqHz = count / 0.2;              // cửa sổ 0.2s
float flowLpm = (freqHz + 5.0) / 190.0;  // Q = (F+5)/190
```

Nên tái dùng cơ chế chống nhiễu (NOISE reject/debounce) đã có trong `TEST_STARTER.ino` cho RPM, vì cảm biến lưu lượng cũng dễ bị nhiễu bởi chính PWM của bơm.

**Nâng cấp vòng điều khiển bơm (Closed-Loop)**:
Với Q đo được thực tế, ECU có thể so sánh **Q_target(RPM)** (tra theo fuel-map ở Mục 3.1) với **Q_actual** rồi hiệu chỉnh PWM bơm bằng vòng điều khiển P/PI đơn giản — thay vì chỉ set PWM cố định theo RPM và giả định lưu lượng đúng như tính toán (open-loop). Đây là bước nâng cấp từ open-loop sang closed-loop thực sự cho hệ thống cấp dầu.

### 3.3 Van Solenoid (Solenoid Valve)

**Cần ít nhất 2 van**:

| Van | Chức năng | Điều khiển |
|-----|-----------|-----------|
| **Van mồi (Kero-glow valve)** | Đóng/mở dầu mồi (dầu nhớt bề mặt để bắt lửa dễ hơn) | On/Off từ GPIO |
| **Van dầu chính (Main fuel valve)** | Đóng/mở dầu chính khi khởi động thành công | On/Off từ GPIO |

**Timeline khởi động**:
- **0–3s**: Van mồi MỞ, van chính ĐÓNG
- **3–4s**: Van chính MỞ (sau khi xác nhận bắt lửa từ EGT)
- **Shutdown**: Cả 2 van ĐÓNG

---

## 4. Quy Trình Lập Trình Chu Trình Khởi Động (Start Sequence)

### 4.1 Sơ Đồ Tổng Quát

```
┌─────────────────────────────────────────────────────────────┐
│                    KHỞI ĐỘNG ĐỘNG CƠ PHẢN LỰC              │
└─────────────────────────────────────────────────────────────┘

Bước 1: GlowPlug / Pre-heat (3–5 giây)
  ├─ Bật điện trở sưởi (Glow plug)
  ├─ Làm nóng buồng mồi
  └─ Chuẩn bị điều kiện để dầu bắt lửa dễ dàng

Bước 2: Spin up / Motor Đề Quay (4–5 giây)
  ├─ Cấp xung PWM tăng dần: 1000µs → 1450µs (mỗi 10ms +1µs)
  ├─ Motor đề quay, tạo luồng gió mồi
  ├─ RPM tăng từ 0 → 10.000 RPM
  └─ Theo dõi: Nếu RPM không tăng → FAIL

Bước 3: Ignition / Mở Dầu Mồi (2–3 giây)
  ├─ Mở van dầu mồi (Solenoid valve)
  ├─ Dầu phun vào buồng mồi, bắt lửa từ glow plug
  ├─ Giám sát EGT: phải tăng lên > 150°C trong 2 giây
  └─ Nếu EGT không tăng → FAIL TO IGNITE

Bước 4: Ramp Up / Tăng Ga (10–15 giây)
  ├─ Motor đề tiếp tục nhận xung (tuy có mắc cáp dầu chính)
  ├─ Mở van dầu chính
  ├─ Tăng tốc độ bơm dầu dần dần (0% → 80%)
  ├─ RPM tăng từ 10.000 → 42.000 RPM (tua garanti Enjet E86)
  ├─ Giám sát EGT: giữ trong vùng 750–820°C
  └─ Quá 850°C → cắt bơm; quá 900°C → shutdown

Bước 5: Disconnect / Ngắt Motor Đề (khi RPM > 35.000)
  ├─ Ngắt hoàn toàn xung cấp cho ESC (1000µs = 0% throttle)
  ├─ Tắt điện trở sưởi (Glow plug)
  ├─ Động cơ chính thức tự vận hành (Sustained)
  └─ Chuyển sang chế độ control vòng lặp (closed-loop RPM + EGT)
```

### 4.2 Pseudocode Chu Trình Khởi Động

```cpp
enum StartState {
  IDLE,
  GLOWPLUG,        // Bước 1
  SPINUP,           // Bước 2
  IGNITION,         // Bước 3
  RAMPUP,           // Bước 4
  SUSTAINED,        // Bước 5
  FAIL
};

class StartSequence {
  StartState state = IDLE;
  unsigned long startTime = 0;
  
  void begin() {
    startTime = millis();
    state = GLOWPLUG;
    digitalWrite(GLOW_PIN, HIGH);  // Bật glow plug
  }
  
  void update() {
    unsigned long elapsed = millis() - startTime;
    
    switch (state) {
      case GLOWPLUG:  // 0–3000ms
        if (elapsed >= 3000) {
          state = SPINUP;
          startTime = millis();
          // Bắt đầu tăng xung motor đề từ 1000µs
        }
        break;
        
      case SPINUP:  // 0–4500ms (tăng từ 1000µs→1450µs)
        {
          int targetPwm = 1000 + (elapsed / 10);  // +1µs mỗi 10ms
          targetPwm = min(targetPwm, 1450);
          escWriteUs(targetPwm);
          
          if (getRpm() < 500 && elapsed > 2000) {
            // Motor không quay → FAIL
            state = FAIL;
            break;
          }
          
          if (targetPwm >= 1450) {
            state = IGNITION;
            startTime = millis();
            digitalWrite(FUEL_PRIME_PIN, HIGH);  // Mở van dầu mồi
          }
        }
        break;
        
      case IGNITION:  // 0–3000ms
        {
          float egtTemp = readEGT();
          
          if (egtTemp > 150.0) {
            // Bắt lửa thành công
            state = RAMPUP;
            startTime = millis();
            digitalWrite(FUEL_MAIN_PIN, HIGH);   // Mở van dầu chính
          } else if (elapsed > 3000) {
            // Không bắt lửa → FAIL
            state = FAIL;
          }
        }
        break;
        
      case RAMPUP:  // 0–15000ms
        {
          float egtTemp = readEGT();
          int rpm = getRpm();
          
          // Điều khiển tốc độ bơm dầu theo RPM
          int pumpPwm = map(rpm, 10000, 42000, 0, 255);
          pumpPwm = constrain(pumpPwm, 0, 255);
          analogWrite(PUMP_PWM_PIN, pumpPwm);
          
          // Giám sát EGT
          if (egtTemp > 850.0) {
            pumpPwm = 0;  // Cắt bơm
          } else if (egtTemp > 900.0) {
            state = FAIL;  // Shutdown
            break;
          }
          
          // Khi RPM vượt 35000, ngắt motor đề
          if (rpm > 35000) {
            state = SUSTAINED;
            startTime = millis();
            escWriteUs(1000);  // 0% throttle = ESC off
            digitalWrite(GLOW_PIN, LOW);  // Tắt glow plug
          }
        }
        break;
        
      case SUSTAINED:
        // Động cơ tự vận hành, chuyển sang closed-loop control
        // Giữ EGT trong 750–820°C bằng điều chỉnh bơm dầu/throttle
        break;
        
      case FAIL:
        // Dừng tất cả
        escWriteUs(1000);
        digitalWrite(GLOW_PIN, LOW);
        digitalWrite(FUEL_PRIME_PIN, LOW);
        digitalWrite(FUEL_MAIN_PIN, LOW);
        analogWrite(PUMP_PWM_PIN, 0);
        break;
    }
  }
};
```

### 4.3 Các Điểm Kiểm Tra An Toàn (Safety Checks)

| Điều kiện | Hành động | Mục đích |
|-----------|----------|---------|
| RPM không tăng sau 2s (Spinup) | → FAIL | Phát hiện motor đề bị tắc/hỏng |
| EGT không tăng sau 3s (Ignition) | → FAIL | Phát hiện lỗi bắt lửa |
| EGT > 850°C | Cắt bơm dầu | Tránh quá nóng |
| EGT > 900°C | Shutdown | Tránh cháy động cơ |
| RPM > 152.000 (quá tua) | Cắt bơm, giảm throttle | Tránh cánh nén bị quá tải |

---

## 5. Các Cải Thiện Phần Cứng Cảm Biến (Optional nhưng Khuyên Dùng)

### 5.1 Bộ Lọc Thông Thấp (Low-Pass Filter) Cho RPM Sensor

**Vấn đề**: Tín hiệu từ cảm biến RPM (KMZ10A) dễ bị nhiễu PWM của motor đề lập lên, gây double-pulse hoặc skip pulse.

**Giải pháp**: Thêm bộ lọc RC ngay sau INA826 (TP_INA_OUT):
```
INA826_OUT ──[R]──┬──TP_INA_OUT → (tầng offset/gain hiện tại)
                  │
                  [C]
                  │
                  GND
                  
Giá trị gợi ý: R = 10kΩ, C = 100nF → f_cắt ≈ 160Hz
(hoặc R = 4.7kΩ, C = 220nF → f_cắt ≈ 150Hz)
```

**Tần số cắt nên nằm giữa**:
- **Tần số PWM motor đề** (thường 1–20kHz)
- **Tần số tín hiệu RPM tối đa** (152.000 RPM × 1 cực / 60 ≈ 2.533kHz)

Lọc 2 tầng (cascade) sẽ tốt hơn 1 tầng → dốc suy giảm mạnh hơn.

### 5.2 Đi Dây Cảm Biến (Cabling Best Practice)

- **Dây SENSOR_P / SENSOR_N**: Dùng **twisted pair (xoắn đôi)**, không dùng dây thẳng song song
- **Tách xa dây nguồn / dây motor đề**: Cách nhau ≥ 10cm (giảm EMI)
- **Dùng shielding** nếu có thể (lưới kim loại quanh dây, nối earth tại 1 đầu)
- **Độ dài dây**: ≤ 1 mét từ cảm biến đến mạch ECU

---

## 6. Tóm Tắt Checklist Thiết Kế ECU

- [ ] **ESC firmware tuning**: Startup Power 0.125–0.25, Motor Timing High
- [ ] **Thuật toán ramping**: Tăng PWM dần (≥ 10ms/µs) thay vì nhảy đột ngột
- [ ] **RPM sensor**: Hoạt động ổn định, không có skip pulse
- [ ] **EGT sensor**: Kết nối MAX6675/MAX31855, đọc nhiệt độ chính xác
- [ ] **Solenoid valves (×2)**: Van mồi + van dầu chính
- [ ] **Fuel pump control**: PWM điều khiển tốc độ bơm theo RPM
- [ ] **Safety checks**: Fail to ignite, over-temp, over-rpm
- [ ] **Closed-loop control loop**: Giữ EGT ở vùng tối ưu 750–820°C

---

## 7. Tài Liệu Tham Khảo Bổ Sung

- **BLHeli_S firmware**: https://github.com/bitdump/BLHeli (cài đặt param cho startup)
- **Enjet E86 specifications**: RPM max 152.000, nhiệt độ max 700°C+
- **MAX6675/MAX31855 datasheet**: Thermocouple amplifier IC
- **KMZ10A AMR sensor datasheet**: Magnetoresistive sensor characteristics
- **ESP32 PWM/LEDC**: Điều khiển ESC, bơm dầu, solenoid qua GPIO/LEDC
