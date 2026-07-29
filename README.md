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

## Tính Năng Nổi Bật (Version 2.4)

- **Giao tiếp Web Serial API**: Không độ trễ mạng, điều khiển mượt mà với thanh trượt trực quan.
- **Chống nhiễu cấp độ Công Nghiệp (CRC-8)**: 
  - Giao thức Serial được bảo vệ bằng checksum CRC-8.
  - Loại bỏ hoàn toàn 100% các lệnh bị lật bit/lỗi do nhiễu từ motor/ESC truyền ngược qua dây UART. 
  - Khóa bảo vệ: Tự động từ chối mọi lệnh không có mã CRC-8 hợp lệ sau khi kết nối Web.
- **Tách biệt Log thông minh**:
  - **Màn hình Serial Log (RX / TX)**: Chuyên hiển thị lệnh điều khiển (`TX ->`) và phản hồi (`[RX OK]`, `[RX NOISE DROPPED]`).
  - **Màn hình Telemetry Log (WEB_DATA)**: Chuyên hiển thị dữ liệu cảm biến theo thời gian thực.
- **Điều khiển độc lập hoàn toàn**: 
  - Bơm nhiên liệu (Pump) và Mô-tơ khởi động (Starter) qua thanh trượt PWM (1000µs - 2000µs).
  - Bật/tắt Van 1, Van 2, Đánh lửa (Igniter).
- **Lưu Cấu Hình (SD Card)**: Lưu thiết lập an toàn và bước nhảy của Pump/Starter vào thẻ nhớ để gọi lại ở lần sau.
- **Dừng Khẩn Cấp (ALL OFF)**: Dừng mọi hoạt động ngay lập tức với 1 nút bấm.
- **Biểu đồ Real-time**: Theo dõi RPM và EGT trực quan trên Web bằng Chart.js.

---

## Sơ Đồ Chân (Pinout) ESP32

| Thiết bị | Chân GPIO | Chú thích |
|---------|----------|----------|
| **Cảm biến nhiệt (MAX31855)** | CLK=18, CS=5, DO=19 | Giao tiếp SPI |
| **Cảm biến vòng tua (RPM)** | 33 | Xung vào từ cảm biến (KMZ10A hoặc Hall) |
| **Bơm nhiên liệu (Pump)** | 26 | Tín hiệu PWM điều khiển ESC |
| **Mô-tơ Đề (Starter)** | 25 | Tín hiệu PWM điều khiển ESC |
| **Van nhiên liệu 1 (Valve 1)** | 17 | Tín hiệu Digital (Relay/Mosfet) |
| **Van khởi động 2 (Valve 2)** | 16 | Tín hiệu Digital (Relay/Mosfet) |
| **Đánh lửa (Igniter)** | 32 | Tín hiệu Digital (Relay/Mosfet) |
| **Thẻ nhớ MicroSD** | CS=13, SCK=14, MOSI=23, MISO=27 | Giao tiếp SPI |

---

## Khắc Phục Sự Cố (Troubleshooting)

1. **Lỗi không kết nối được Web Serial:**
   - Trình duyệt báo `The port is already open`: Đảm bảo bạn đã đóng Serial Monitor trong Arduino IDE. Bạn có thể sử dụng nút "❌ Ngắt Kết Nối" trên Web để nhả cổng COM khi cần nạp lại code.
   - Trình duyệt không có nút kết nối: Hãy dùng Chrome hoặc Edge bản mới nhất. Firefox/Safari hiện không hỗ trợ Web Serial API.

2. **Log báo `[RX NOISE DROPPED]` liên tục:**
   - Điều này là bình thường nếu mạch chịu nhiễu điện từ nặng (từ ESC/Motor). Firmware đã tự động chặn các lệnh lỗi này lại nhờ thuật toán CRC-8.
   - Để cải thiện phần cứng: Dùng dây nối đất (GND) chung và to hơn, xoắn dây UART, hoặc sử dụng lõi Ferrite chống nhiễu.

3. **Thông báo lỗi EGT `ERR(SHORT_VCC)` hoặc `ERR(OPEN)`:**
   - Kiểm tra lại dây Thermocouple (MAX31855). Dây bị đứt, lỏng, hoặc chạm vỏ máy.

4. **Van nhiên liệu/Đánh lửa không hoạt động:**
   - Kiểm tra nguồn cấp riêng cho Relay/Mosfet. Chân tín hiệu từ ESP32 chỉ có 3.3V, không thể cấp nguồn trực tiếp cho cuộn dây Relay hoặc bộ tạo tia lửa.

---

## Liên Hệ & Đóng Góp

- Mọi đóng góp đều được chào đón. Xem thêm tại [GitHub Issues](https://github.com/phamthemy3089/Ardu_ECU/issues).
