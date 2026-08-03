# Ardu_ECU — ECU Manual V1 (Web Serial Dashboard)

ECU (Engine Control Unit) cho động cơ phản lực mô hình, chạy trên nền ESP32.
Phiên bản **Manual V1** tập trung vào điều khiển thủ công hoàn toàn qua **Web Serial API** (không dùng WiFi), đảm bảo độ tin cậy và chống nhiễu cao nhất trong môi trường công nghiệp/motor.

---

## Cấu Trúc Project

```
Ardu_ECU/
│
├── PROJECT_CURRENT/           ← Dự án đang phát triển (bắt đầu từ đây)
│   ├── Firmware/              
│   │   └── ECU_ManualV1/
│   │       └── ECU_ManualV1.ino   ← Code ESP32 (Mở bằng Arduino IDE)
│   └── Tools/                 
│       └── index.html             ← Web Dashboard (Mở bằng Chrome/Edge)
│
├── TEST/                      ← Firmware test riêng lẻ
├── REFERENCES/                ← Tài liệu và mã nguồn tham khảo cũ
└── README.md
```

---

## Bắt Đầu Nhanh

### 1. Upload Firmware
- **Thư mục**: `PROJECT_CURRENT/Firmware/ECU_ManualV1/ECU_ManualV1.ino`
- **Board**: ESP32 Dev Module (hoặc NodeMCU-32S)
- **Baudrate Serial**: 115200

### 2. Sử Dụng Web Dashboard
- Không cần kết nối WiFi. Chỉ cần cắm cáp USB vào máy tính.
- Mở file `PROJECT_CURRENT/Tools/index.html` bằng trình duyệt **Google Chrome** hoặc **Microsoft Edge** (hỗ trợ Web Serial API).
- Bấm nút **"🔌 Kết Nối USB COM"** trên góc phải và chọn cổng COM của ESP32.

---

## Tính Năng Nổi Bật (Version 5.8)

- **Thuật Toán Gia Tốc Mượt Bơm (Pump S-Curve Smoothstep Ramp 1.5s)**: Gia tốc ga nhiên liệu mượt mà theo đa thức bậc 3 ($3x^2 - 2x^3$), giúp động cơ rú ga mượt hệt như máy bay thật, chống sặc dầu & dập lửa.
- **Thuật Toán Gia Tốc Đề (Starter Exponential Ramp Up 1.0s)**: Gia tốc lực kéo mô-tơ Đề theo đường cong mũ lũy thừa, thắng lực ma sát nghỉ ban đầu mà không làm xóc nhông Bendix.
- **Giao tiếp Web Serial API & Mã Hóa CRC-8**: Điều khiển mượt mà, loại bỏ 100% các lệnh bị lật bit do nhiễu UART. Bổ sung **Toast Banner Tiếng Việt** sinh động.
- **Hiển Thị Tín Hiệu (Signal & SD)**: Web UI phân tích trực tiếp trạng thái tín hiệu RPM (`SIG=OK/REST/ERROR`) và tình trạng thẻ nhớ SD.
- **Chuỗi Bộ Lọc RPM 7 Tầng Nâng Cao**: 
  1. *Min Pulse 330µs*: Chặn xung siêu ngắn.
  2. *Dynamic Mask 66.7%*: Siết chặt vùng chết dựa trên chu kỳ ngắt trước đó, chống nhiễu tia lửa điện.
  3. *Median-5*: Sử dụng Sorting Network 5 phần tử (IRAM optimized), triệt tiêu hoàn toàn nhiễu chùm.
  4. *Dynamic Outlier Guard*: Ngưỡng trần linh hoạt theo RPM hiện tại.
  5. *Rate Limiter*: Giới hạn tốc độ biến thiên RPM tối đa (Slew-rate).
  6. *Adaptive PWM-Aware Learning*: Tự động học bản đồ quan hệ PWM ↔ RPM. Có logic tha bổng Jitter thông minh để học ngay cả khi nhiễu chổi than lắt nhắt.
  7. *Cascaded Dual-EMA Bậc 2*: Bộ lọc LPF 2 tầng nối tiếp với hệ số lọc động, êm ái mọi dải ga.
- **Bảo Vệ Chống Bùng Nhiệt Cấp Công Nghiệp (`PEGT > 740°C`)**: Tự động ngắt Bơm & xả mát buồng đốt. Tích hợp kẹp giới hạn biên độ biến thiên nhiệt (Gradient Clamping ±600°C/s) và trì hoãn xác nhận (Debouncing 2 chu kỳ) để chống nhiễu đỉnh (Spike).
- **Khóa An Toàn Liên Động 2 Chiều (Pump-Valve Interlock)**: Ngăn chặn tự động bật bơm khi khóa van, hoặc tự động ngắt bơm khi 2 van nhiên liệu đóng.
- **Dừng Khẩn Cấp Thông Minh**:
  - Nhấp nút vật lý (hoặc phím Space): Dừng Bơm/Lửa/Van, tự động giữ Đề thổi khí nóng.
  - Nhấn giữ nút 3s (hoặc Shift+Space): Dừng toàn bộ (Total Shutdown).
- **Calib ESC Dễ Dàng**: Gỡ bỏ lệnh `esccal` xung đột cũ. Tự do Calib ESC bằng cách gạt tắt công tắc Ramp trên Web UI, kéo thanh trượt lên 2000µs và cấp nguồn.

---

## Sơ Đồ Chân (Pinout) ESP32

| Thiết bị | Chân GPIO | Chú thích |
|---------|----------|----------|
| **Cảm biến nhiệt (MAX31855)** | CLK=18, CS=5, DO=19 | Giao tiếp SPI (VSPI) |
| **Cảm biến vòng tua (RPM)** | 33 | Xung vào từ cảm biến (KMZ10A hoặc Hall) |
| **Bơm nhiên liệu (Pump)** | 26 | Tín hiệu PWM điều khiển ESC (LEDC CH0) |
| **Mô-tơ Đề (Starter)** | 25 | Tín hiệu PWM điều khiển ESC (LEDC CH1) |
| **Van nhiên liệu 1 (Valve 1)** | 17 | Tín hiệu Digital (Relay/Mosfet) |
| **Van khởi động 2 (Valve 2)** | 16 | Tín hiệu Digital (Relay/Mosfet) |
| **Đánh lửa (Igniter)** | 32 | Tín hiệu Digital (Relay/Mosfet) |
| **Thẻ nhớ MicroSD** | CS=13, SCK=14, MOSI=23, MISO=27 | Giao tiếp SPI (HSPI) |

---

## Khắc Phục Sự Cố (Troubleshooting)

1. **Lỗi không kết nối được Web Serial:**
   - Trình duyệt báo `The port is already open`: Đảm bảo bạn đã đóng Serial Monitor trong Arduino IDE. Sử dụng nút "❌ Ngắt Kết Nối" trên Web để nhả cổng COM trước khi upload code.
   - Trình duyệt không có nút kết nối: Hãy dùng Chrome hoặc Edge bản mới nhất.

2. **Log báo `[RX NOISE DROPPED]` liên tục:**
   - Bình thường khi mạch chịu nhiễu điện từ nặng từ ESC/Motor. Firmware đã chặn các lệnh lỗi nhờ CRC-8.
   - Khắc phục phần cứng: Tiếp đất (GND) chung, xoắn cặp dây UART, sử dụng lõi Ferrite chống nhiễu.

3. **Thông báo lỗi EGT `ERR(SHORT_VCC)` hoặc `ERR(OPEN)`:**
   - Kiểm tra lại dây Thermocouple K (MAX31855). Dây bị đứt, lỏng, hoặc chạm vỏ máy.

4. **Lệnh debug RPM qua Serial:**
   - `rpmlearn`: Xem bảng 20 bin PWM ↔ RPM đã tự động học được.
   - `rpmlearn reset`: Xóa dữ liệu học và bắt đầu học lại.

---

## Liên Hệ & Đóng Góp

- Mọi đóng góp đều được chào đón. Xem thêm tại [GitHub Issues](https://github.com/phamthemy3089/Ardu_ECU/issues).
