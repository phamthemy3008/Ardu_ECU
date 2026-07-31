# PROJECT_CURRENT — ECU Manual V1 (Version 5.4)

**Trạng thái**: Đang phát triển tích cực (Phiên bản v5.4)  
**Cập nhật**: 2026-07-31  

---

## Cấu Trúc Thư Mục

```
PROJECT_CURRENT/
│
├── Firmware/ECU_ManualV1/
│   └── ECU_ManualV1.ino           ← Firmware ESP32 (Arduino IDE, v5.4)
│
├── Tools/
│   └── index.html                 ← Web Dashboard (Chrome/Edge, Web Serial API, v5.4)
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

## Tính Năng Chính (v5.4)

- **Thuật Toán Gia Tốc Mượt Bơm (Pump S-Curve Smoothstep Ramp 1.5s)**: Gia tốc ga nhiên liệu mượt mà theo đa thức bậc 3 ($3x^2 - 2x^3$), giúp động cơ rú ga mượt hệt như máy bay thật, chống sặc dầu & dập lửa. Có công tắc Bật/Tắt tùy chọn độc lập trên Web UI (`set pumpramp on|off`).
- **Thuật Toán Gia Tốc Đề (Starter Exponential Ramp Up 1.0s)**: Gia tốc lực kéo mô-tơ Đề theo đường cong mũ lũy thừa, thắng lực ma sát nghỉ ban đầu mà không làm xóc nhông Bendix hay kẹt motor. Có công tắc Bật/Tắt tùy chọn độc lập trên Web UI (`set starterramp on|off`).
- **Web Serial API & CRC-8**: Điều khiển mượt mà, chống nhiễu lệnh UART bằng mã CRC-8.
- **Bộ Lọc RPM 7 Tầng**: Min Pulse → Dynamic Mask 75% → Median-5 → Dynamic Outlier Guard → Rate Limiter → Adaptive PWM-Aware Learning (Monotonicity Floor + Hysteresis) → Cascaded Dual-EMA Bậc 2.
- **Dừng Khẩn Cấp Thổi Khí Nóng (`estop`)**: Ngắt Bơm/Lửa/Van nhưng tự động bật/giữ Mô-tơ Đề 1300 µs để làm mát buồng đốt khi EGT > 80°C.
- **Nút Cứng Vật Lý Đa Năng (`BTN1` / IO22)**:
  - Nhấp 1 lần (< 3s): Kích hoạt Dừng Khẩn Cấp làm mát.
  - Nhấn giữ 3s (>= 3s): Dừng Toàn Bộ (Full Shutdown, ngắt cả Đề).
- **Khóa An Toàn 2 Chiều (Pump-Valve Interlock)**:
  - Không cho bật Bơm khi cả 2 van nhiên liệu đang ĐÓNG (`ERR:E01`).
  - Tự động ngắt Bơm khi cả 2 van bị đóng (`EV:A02`).
  - Tự động đóng cả 2 van khi tắt Bơm (`EV:A01`).
- **Bảo Vệ Chống Bùng Nhiệt (`PEGT > 740°C`)**: Tự động ngắt Bơm & xả mát khi nhiệt độ dự báo 3s vượt 740°C (`ERR:E02`).
- **Tối Ưu Tài Nguyên ESP32 (Compact Event/Error Codes)**: ECU chỉ phát các mã siêu nhẹ (`EV:A01`..`EV:A05`, `ERR:E01`..`ERR:E02`) qua Serial; Web UI tự động dịch thành Thẻ Banner Thông Báo Tiếng Việt nổi bật.
- **Heartbeat Status LED (IO2)**: Nháy 1Hz báo hiệu ECU đang hoạt động.

---

## Bảng Mã Sự Kiện & Mã Lỗi (Event Code Dictionary)

| Mã Serial | Loại | Ý Nghĩa | Thông Báo Tiếng Việt Hiển Thị Trên Web Dashboard |
|-----------|------|---------|--------------------------------------------------|
| `ERR:E01` | Cảnh báo | Bật Bơm khi cả 2 van đóng | `⚠️ CẢNH BÁO AN TOÀN: Cả 2 van nhiên liệu đều đang ĐÓNG! Vui lòng mở ít nhất 1 van trước.` |
| `ERR:E02` | Khẩn cấp | Quá nhiệt dự báo PEGT > 740°C | `🛑 CẢNH BẢO QUÁ NHIỆT: Dự báo PEGT 3s vượt 740°C! Đã tự động NGẮT BƠM & THỔI LÀM MÁT.` |
| `EV:A01`  | Thông tin | Tắt Bơm -> Tự đóng 2 van | `ℹ️ TỰ ĐỘNG KHÓA: Đã Tắt Bơm -> Tự động đóng cả 2 Van nhiên liệu (V1 & V2).` |
| `EV:A02`  | Thông tin | Đóng 2 van -> Tự ngắt Bơm | `ℹ️ TỰ ĐỘNG KHÓA: Đã đóng cả 2 Van -> Tự động ngắt Bơm nhiên liệu.` |
| `EV:A03`  | Khẩn cấp | Dừng Khẩn Cấp -> Thổi khí 1300µs | `🛑 DỪNG KHẨN CẤP: Ngắt Bơm & Lửa/Van -> Tự động bật Đề 1300µs thổi khí làm mát.` |
| `EV:A04`  | Thông tin | Dừng Khẩn Cấp -> Máy nguội <= 80°C | `ℹ️ DỪNG KHẨN CẤP: Đã ngắt Bơm & Lửa/Van (Máy đã nguội <= 80°C).` |
| `EV:A05`  | Khẩn cấp | Nút 3s / Dừng Toàn Bộ | `⚡ DỪNG TOÀN BỘ: Đã ngắt 100% tất cả thiết bị (Bơm, Đề, Lửa, Van).` |

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
| User Button (BTN1) | 22 (Active LOW) |
| Status LED | 2 (Heartbeat 1Hz) |
| GSU / Debug UART | 1 (TX) / 3 (RX) |
| RC Input 1 / 2 | 34 / 35 |

---

## Cảnh Báo An Toàn

- ESP32 chỉ chịu **3.3V** — dùng level converter cho tín hiệu 5V
- Chỉ dùng **thermocouple loại K**
- Cảm biến RPM KMZ10A cần hiệu chỉnh trimpot **RP1/RP2/RP3** trước khi dùng
- ESP32 cần nguồn điện riêng (không cấp qua USB khi motor chạy)

