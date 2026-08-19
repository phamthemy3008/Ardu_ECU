# Ardu_ECU — ECU Manual V1 (Version 5.9)

ECU (Engine Control Unit) cho động cơ phản lực mô hình mini (Enjet E86), chạy trên vi điều khiển ESP32.
Phiên bản **Manual V1** tập trung vào điều khiển thủ công hoàn toàn qua **Web Serial API** (không dùng WiFi), đảm bảo độ tin cậy và chống nhiễu cao nhất trong môi trường công nghiệp/motor.

---

## Cấu Trúc Project

```
Ardu_ECU/
│
├── PROJECT_CURRENT/           ← Dự án đang phát triển (bắt đầu từ đây)
│   ├── Firmware/              
│   │   └── ECU_ManualV1/
│   │       └── ECU_ManualV1.ino   ← Code ESP32 (v5.9, mở bằng Arduino IDE)
│   ├── Tools/                 
│   │   └── index.html             ← Web Dashboard (v5.9, mở bằng Chrome/Edge)
│   ├── Hardware/                  ← Sơ đồ nguyên lý và Netlist PCB
│   └── Docs/                      ← Tài liệu kỹ thuật & Hướng dẫn triển khai
│
├── TEST/                      ← Firmware test riêng lẻ
├── REFERENCES/                ← Tài liệu và mã nguồn tham khảo
└── README.md
```

---

## Hướng Dẫn Chạy Web Dashboard Trên Windows 11 (Không Cần Cài Thêm Gì)

Web Dashboard được thiết kế theo chuẩn **Standalone Native HTML5 / JS Vanilla & Web Serial API**.

> [!IMPORTANT]
> **Yêu cầu**: Máy tính **Windows 11** + Trình duyệt **Google Chrome** hoặc **Microsoft Edge**.  
> **KHÔNG CẦN CÀI THÊM PHẦN MỀM GÌ**: Không cần cài Node.js, Python, hay web server. Chạy hoàn toàn offline (Zero Dependencies).

1. **Thao tác khởi chạy (Chế độ mặc định - Không cần cài Node.js)**:
   - Mở thư mục `PROJECT_CURRENT/Tools/` trong File Explorer.
   - Click đúp chuột vào file `index.html` (hoặc nhấp chuột phải chọn *Open with -> Google Chrome / Microsoft Edge*).
   - Trình duyệt sẽ mở trực tiếp file `file:///C:/.../index.html`.

2. **Kết nối Serial qua USB**:
   - Cắm cáp USB nối ESP32 với máy tính.
   - Bấm nút **"🔌 Kết Nối USB COM"** ở góc phải Web Dashboard.
   - Chọn đúng cổng COM của ESP32 (ví dụ: `COM3`, `COM4` - chip `CP210x` hoặc `CH340`) và bấm **Connect**.
   - Trạng thái sẽ báo `CỔNG COM ĐÃ MỞ (115200 baud, CRC-8)` và truyền/nhận dữ liệu lập tức.

3. **Khả năng sử dụng tính năng AI khi chạy Local**:
   - **Khi mở trực tiếp `index.html` (`file:///...`)**: Tất cả tính năng điều khiển Bơm/Đề/Van, Đồ thị Telemetry, Dừng khẩn cấp, Bảng Cài đặt Ngưỡng an toàn & Lưu `localStorage` đều **hoạt động 100% hoàn hảo không cần mạng**. Tuy nhiên, nút **"🧠 Phân Tích Dữ Liệu Log & Telemetry AI"** sẽ báo lỗi do không có server backend trung gian để gọi Gemini API.
   - **Để bật đầy đủ tính năng Chẩn Đoán AI khi chạy ở máy local**:
     1. Cài đặt **Node.js** (Lựa chọn thêm nếu muốn dùng AI).
     2. Tạo file `.env` tại thư mục gốc với nội dung `GEMINI_API_KEY=mã_api_key_của_bạn` (lấy miễn phí tại Google AI Studio).
     3. Mở Terminal / CMD tại thư mục dự án và chạy lệnh: `npm start` (hoặc `node server.js`).
     4. Mở trình duyệt truy cập `http://localhost:3000` -> Sử dụng đầy đủ tính năng phân tích & chẩn đoán sự cố tự động bằng Gemini AI!

---

## Bắt Đầu Với Firmware ESP32

1. **Upload Firmware**:
   - Mở file `PROJECT_CURRENT/Firmware/ECU_ManualV1/ECU_ManualV1.ino` bằng Arduino IDE.
   - Chọn Board **ESP32 Dev Module**, Baudrate 115200.
   - Thư viện cần: `Adafruit_MAX31855`, `SD` (xung PWM được điều khiển trực tiếp qua bộ tạo xung LEDC phần cứng của ESP32).

---

## Tính Năng Nổi Bật (Version 5.9)

- **Ghi Log Telemetry & Sự Kiện Lên Thẻ SD (`/ECUxxx.CSV`)**:
  - Tự động ghi nhật ký định kỳ 2Hz và hàng đợi sự kiện lỗi (`sdLogEvent`) ra file CSV trên thẻ MicroSD.
  - Các lệnh quản lý log qua Serial: `sdstatus`, `sdtest`, `set sdlog on|off`.

- **Tùy Chỉnh Ngưỡng An Toàn Trên Web UI (Safety Thresholds)**:
  - Cho phép thiết lập và lưu trên giao diện Web các ngưỡng: **PEGT 3s Max**, **EGT Max**, và **Max RPM**.
  - Dữ liệu được lưu vào **localStorage** của trình duyệt, tự động đồng bộ tới ECU khi kết nối và lưu vào `/ECUCFG.TXT` trên thẻ SD.

- **Cảnh Báo & Bảo Vệ An Toàn Tự Động**:
  - `ERR:E06` — Bảo vệ Quá Vòng Tua (Overrev) khi RPM vượt `Max RPM`.
  - `ERR:E02` — Bảo vệ Quá Nhiệt (Overheat) khi PEGT 3s hoặc EGT trực tiếp vượt ngưỡng cài đặt.
  - Khóa liên động 2 chiều Bơm - Van (`ERR:E01`, `EV:A01`, `EV:A02`).
  - Dừng khẩn cấp thổi khí làm mát buồng đốt (`estop` / `EV:A03`).
  - Nút cứng vật lý đa năng `BTN1` (IO22): Nhấp < 3s -> Dừng Khẩn Cấp làm mát; Giữ >= 3s -> Dừng Toàn Bộ (Full Shutdown).

- **Thuật Toán Gia Tốc Mượt Bơm & Đề (Smooth Ramps)**:
  - Pump Smoothstep S-Curve Ramp (1.5s): Đẩy ga nhiên liệu mượt chống dập lửa.
  - Starter Exponential Ramp (1.0s): Tăng ga đề theo đường cong mũ thắng ma sát nghỉ.
  - Công tắc Bật/Tắt ramp độc lập trên Web UI (`set pumpramp on|off`, `set starterramp on|off`).

- **Bộ Lọc RPM 7 Tầng Chuyên Sâu & Giao Thức Bảo Mật CRC-8**:
  - Loại bỏ hoàn toàn nhiễu cao tần từ ESC/Motor và đảm bảo tính toàn vẹn tín hiệu UART.

---

## Sơ Đồ Chân (Pinout) ESP32

| Thiết bị | Chân GPIO | Chú thích |
|---------|----------|----------|
| **Cảm biến nhiệt (MAX31855)** | CLK=18, CS=5, DO=19 | Giao tiếp SPI (VSPI) |
| **Cảm biến vòng tua (RPM)** | 33 | Cảm biến từ/quang (KMZ10A / Hall) |
| **Bơm nhiên liệu (Pump)** | 26 | Tín hiệu PWM điều khiển ESC (LEDC CH0) |
| **Mô-tơ Đề (Starter)** | 25 | Tín hiệu PWM điều khiển ESC (LEDC CH1) |
| **Van nhiên liệu 1 (Valve 1)** | 17 | Tín hiệu Digital (Relay/Mosfet) |
| **Van khởi động 2 (Valve 2)** | 16 | Tín hiệu Digital (Relay/Mosfet) |
| **Đánh lửa (Igniter)** | 32 | Tín hiệu Digital (Relay/Mosfet) |
| **Thẻ nhớ MicroSD** | CS=13, SCK=14, MOSI=23, MISO=27 | Giao tiếp SPI (HSPI) |
| **Nút nhấn vật lý (BTN1)** | 22 | Nút đa năng Dừng Khẩn Cấp / Shutdown |
| **Đèn LED Trạng Thái** | 2 | Nháy Heartbeat 1Hz |


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

4. **Lệnh debug RPM mới qua Serial:**
   - `rpmlearn`: Xem bảng 20 bin PWM ↔ RPM đã tự động học được.
   - `rpmlearn reset`: Xóa dữ liệu học và bắt đầu học lại.

---

## Liên Hệ & Đóng Góp

- Mọi đóng góp đều được chào đón. Xem thêm tại [GitHub Issues](https://github.com/phamthemy3089/Ardu_ECU/issues).
