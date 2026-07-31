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

## Tính Năng Nổi Bật (Version 4.2)

- **Giao tiếp Web Serial API**: Không độ trễ mạng, điều khiển mượt mà với thanh trượt trực quan.
- **Bảo mật giao thức CRC-8**: 
  - Giao thức Serial được bảo vệ bằng checksum CRC-8.
  - Loại bỏ hoàn toàn 100% các lệnh bị lật bit/lỗi do nhiễu từ motor/ESC truyền ngược qua dây UART. 
  - Khóa bảo vệ: Tự động từ chối mọi lệnh không có mã CRC-8 hợp lệ sau khi kết nối Web.
- **Chuỗi Bộ Lọc RPM 7 Tầng Chuyên Sâu (DSP & Physical Modeling)**:
  1. *Min Pulse 330µs*: Chặn xung siêu ngắn dưới ngưỡng cảm biến.
  2. *Dynamic Mask 75%*: Siết chặt vùng chết dựa trên chu kỳ ngắt trước đó.
  3. *Median-5*: Sử dụng Sorting Network 5 phần tử (IRAM optimized), triệt tiêu hoàn toàn nhiễu chùm (Burst Noise 2 xung rác liên tiếp).
  4. *Dynamic Outlier Guard*: Ngưỡng trần linh hoạt theo RPM hiện tại, ngăn xung vọt 50k-80k khi ngắt Đề.
  5. *Rate Limiter*: Giới hạn tốc độ biến thiên RPM tối đa (Slew-rate), loại bỏ các cú nhảy RPM bất thường.
  6. *Adaptive PWM-Aware Learning + Monotonicity Constraint*:
     - Tự động học bản đồ quan hệ PWM ↔ RPM ở 20 bins (50µs/bin) khi tín hiệu sạch.
     - Ràng buộc vật lý đơn điệu (Monotonicity Floor): PWM cao hơn bắt buộc RPM phải lớn hơn hoặc bằng mức đã học của PWM thấp hơn.
     - Bảo vệ chuyển tiếp 2s sau khi tắt Đề: Đảm bảo RPM khi động cơ tự quay / coasting không bị ngắt rụp về 0.
  7. *Cascaded Dual-EMA Bậc 2*: Bộ lọc LPF 2 tầng nối tiếp, tự động điều chỉnh hệ số lọc (Alpha) theo trạng thái Đề, Chuyển tiếp & Tốc độ quay của động cơ.
- **Tách biệt 3 Luồng RPM**:
  - `RRPM`: Vòng tua thô nguyên thủy 100% (Unfiltered) dùng làm mốc đối chứng.
  - `FRPM`: Vòng tua lọc an toàn chuyên dùng cho Trigger.
  - `RPM`: Vòng tua lọc mượt 2 tầng hiển thị trực quan lên đồ thị Web.
- **Dự báo Nhiệt độ 3s (`PEGT`)**: Tính toán tốc độ tăng nhiệt `dEGT` (°C/s) và dự báo nhiệt độ EGT sau 3 giây để cảnh báo nguy cơ Hot-Start.
- **Tách biệt Log thông minh**:
  - **Màn hình Serial Log (RX / TX)**: Chuyên hiển thị lệnh điều khiển và phản hồi.
  - **Màn hình Telemetry Log (WEB_DATA)**: Chuyên hiển thị dữ liệu cảm biến & tiến trình `ALEARN`.
- **Điều khiển & Cấu hình**:
  - Điều khiển độc lập Bơm (Pump), Mô-tơ Đề (Starter), Van 1, Van 2, Đánh lửa (Igniter).
  - Lưu và nạp cấu hình thông số cài đặt từ thẻ nhớ MicroSD (`savecfg` / `loadcfg`).
  - Nút Dừng Khẩn Cấp `ALL OFF` (Phím tắt `Space`).

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

4. **Lệnh debug RPM mới qua Serial:**
   - `rpmlearn`: Xem bảng 20 bin PWM ↔ RPM đã tự động học được.
   - `rpmlearn reset`: Xóa dữ liệu học và bắt đầu học lại.

---

## Liên Hệ & Đóng Góp

- Mọi đóng góp đều được chào đón. Xem thêm tại [GitHub Issues](https://github.com/phamthemy3089/Ardu_ECU/issues).
