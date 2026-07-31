# PROJECT_CURRENT — ECU Manual V1 (Version 4.3)

**Trạng thái**: Đang phát triển tích cực  
**Cập nhật**: 2026-07-31

---

## Cấu Trúc Thư Mục

```
PROJECT_CURRENT/
│
├── Firmware/ECU_ManualV1/
│   └── ECU_ManualV1.ino           ← Firmware ESP32 (Arduino IDE)
│
├── Tools/
│   └── index.html                 ← Web Dashboard (Chrome/Edge, Web Serial API)
│
├── Hardware/
│   ├── ECU_JET_20260723.net       ← Netlist PCB (PADS)
│   └── SCH_MinijetengineECU_20260723.json ← Sơ đồ mạch ECU (EasyEDA JSON)
│
├── Docs/
│   ├── JETENGINE_ECU_DESIGN_REFERENCE.md  ← Tài liệu thiết kế ECU
│   ├── STARTER_MOTOR_IMPLEMENTATION_GUIDE.md  ← Hướng dẫn triển khai Starter
│   └── BringUp_Logs/              ← Dữ liệu đo RPM thực tế (DSO152, Serial)
│
└── README.md                      ← File này
```

---

## Bắt Đầu Nhanh

### Upload Firmware
1. Mở `Firmware/ECU_ManualV1/ECU_ManualV1.ino` bằng **Arduino IDE**
2. Board: **ESP32 Dev Module** (hoặc NodeMCU-32S)
3. Thư viện cần: `Adafruit_MAX31855`, `SD` (ESC PWM dùng LEDC có sẵn — **không** cần `ESP32Servo`, xem `CLAUDE.md`)
4. Upload → Baudrate 115200

### Sử Dụng Web Dashboard
- **Không dùng WiFi.** Kết nối qua **USB Serial** trực tiếp.
- Mở `Tools/index.html` bằng **Google Chrome** hoặc **Microsoft Edge**.
- Bấm **"🔌 Kết Nối USB COM"** → chọn cổng COM của ESP32.

---

## Tính Năng Chính (v4.3)

- **Web Serial API**: Điều khiển mượt mà, không độ trễ mạng
- **Bảo mật CRC-8**: Chống nhiễu lệnh từ motor/ESC qua UART
- **Bộ Lọc RPM 7 Tầng**: Min Pulse → Dynamic Mask 75% → Median-5 → Dynamic Outlier Guard → Rate Limiter → Adaptive PWM-Aware Learning (Monotonicity Constraint + 2s Hysteresis) → Cascaded Dual-EMA Bậc 2
- **3 Luồng RPM**: `RRPM` (thô 100%), `FRPM` (trigger), `RPM` (hiển thị)
- **Dự báo EGT 3s** (`PEGT`): Cảnh báo Hot-Start
- **Lưu/nạp cấu hình SD Card**: `savecfg` / `loadcfg`

---

## Pinout ESP32

| Chức năng | GPIO |
|-----------|------|
| MAX31855 CLK/CS/DO | 18 / 5 / 19 |
| RPM Sensor | **33** |
| Pump ESC | 26 (LEDC CH0) |
| Starter ESC | 25 (LEDC CH1) |
| Valve 1 / Valve 2 | 17 / 16 |
| Ignition/Glow | 32 |
| SD CS/SCK/MOSI/MISO | 13 / 14 / 23 / 27 |

---

## Cảnh Báo An Toàn

- ESP32 chỉ chịu **3.3V** — dùng level converter cho tín hiệu 5V
- Chỉ dùng **thermocouple loại K**
- Cảm biến RPM KMZ10A cần hiệu chỉnh trimpot **RP1/RP2/RP3** trước khi dùng
- ESP32 cần nguồn điện riêng (không cấp qua USB khi motor chạy)
