# PROJECT_CURRENT — ECU Manual V1 (Version 8.0)

**Trạng thái**: Phiên bản v8.0 (Hoàn thiện & Ổn định)  
**Cập nhật**: 2026-08-24  

---

## Cấu Trúc Thư Mục

```
PROJECT_CURRENT/
│
├── Firmware/ECU_ManualV1/
│   └── ECU_ManualV1.ino           ← Firmware ESP32 (Arduino IDE / GitHub Actions, v8.0)
│
├── Tools/
│   └── index.html                 ← Web Dashboard & Web Serial ESP32 Flasher (Chrome/Edge, v8.0)
│
├── Hardware/
│   ├── 3D Printable Files/        ← File in 3D (Bendix Sleeve, Giá đỡ Motor Đề)
│   │   ├── Bendix Sleeve_10mm_NoSpring_Reinforced.stl   ← Ống áo Bendix cốt 10mm (Tự văng không lò xo)
│   │   ├── Bendix Sleeve_10.4mm_NoSpring_Reinforced.stl ← Ống áo Bendix cốt 10.4mm (Tự văng không lò xo)
│   │   ├── Bendix Sleeve_Reinforced.stl                ← Ống áo Bendix cốt 9mm (Bản gia cường)
│   │   ├── Bendix Sleeve.stl                           ← Ống áo Bendix cốt 9mm (Bản gốc)
│   │   ├── Brushless motor mount.stl                   ← Giá đỡ motor không chổi than
│   │   └── Brushed motor mount.stl                     ← Giá đỡ motor chổi than
│   ├── ECU_JET_20260723.net       ← Netlist PCB (PADS)
│   └── SCH_MinijetengineECU_20260723.json ← Sơ đồ mạch ECU (EasyEDA JSON)
│
├── Docs/
│   ├── JETENGINE_ECU_DESIGN_REFERENCE.md  ← Tài liệu thiết kế ECU
│   ├── STARTER_MOTOR_IMPLEMENTATION_GUIDE.md  ← Hướng dẫn triển khai Starter
│   ├── CHANGELOG.md               ← Nhãn lịch sử các phiên bản
│   └── BringUp_Logs/              ← Dữ liệu đo RPM thực tế (DSO152, Serial)
│
└── README.md                      ← File hướng dẫn này
```

---

## Hướng Dẫn Chạy Web Dashboard Trên Windows 11 (Không Cần Cài Thêm Gì)

Ứng dụng Web Dashboard được thiết kế theo tiêu chuẩn **Standalone Native HTML5 & Web Serial API**.

> [!IMPORTANT]
> **Yêu cầu hệ thống**: Máy tính chạy **Windows 11** + Trình duyệt **Google Chrome** hoặc **Microsoft Edge** (đã tích hợp sẵn Web Serial API từ phiên bản 89 trở lên).  
> **TUYỆT ĐỐI KHÔNG CẦN CÀI THÊM GÌ**: Không cần Node.js, không cần Python, không cần cài server web, không cần cài thêm thư viện phụ trợ (Zero Dependencies / Offline Ready).

### Các bước khởi chạy Web Dashboard:

1. **Khởi chạy giao diện (Chế độ mặc định - Offline 100% không cần Node.js)**:
   - Mở thư mục `PROJECT_CURRENT/Tools/` trên máy tính.
   - **Click đúp chuột** vào file `index.html` (hoặc chuột phải chọn *Open with -> Google Chrome / Microsoft Edge*).
   - Trình duyệt sẽ mở trực tiếp file dạng `file:///C:/.../PROJECT_CURRENT/Tools/index.html`.

2. **Kết nối USB COM với ESP32**:
   - Cắm cáp USB (MicroUSB / Type-C) nối board ESP32 vào máy tính Windows 11.
   - Trên giao diện Web Dashboard, bấm nút **"🔌 Kết Nối USB COM"** ở góc trên bên phải.
   - Trong cửa sổ popup của trình duyệt, chọn đúng cổng COM tương ứng của ESP32 (ví dụ: `COM3`, `COM4` - thường ghi tên chip USB `CP210x` hoặc `CH340`).
   - Bấm **Connect**. Màn hình console sẽ báo `CỔNG COM ĐÃ MỞ (115200 baud, CRC-8)` và toàn bộ dữ liệu telemetry sẽ cập nhật thời gian thực.

3. **Tính Năng Nạp Firmware Trực Tiếp Trên Browser (Web Serial ESP32 Flasher)**:
   - Web Dashboard tích hợp thư viện `esptool-js` giúp người dùng nạp file `firmware.bin` trực tiếp vào ESP32 thông qua trình duyệt mà không cần cài đặt Python hay Arduino IDE.
   - Bấm nút **"🔥 Nạp Firmware USB"**, chọn file `firmware.bin`, đưa ESP32 về chế độ Flash (nếu cần) và bấm **Program Firmware**.

4. **Khả năng sử dụng tính năng Chẩn Đoán AI khi chạy ở máy Local**:
   - **Chạy trực tiếp HTML (`file:///...`)**: Tất cả các tính năng cốt lõi (điều khiển Bơm/Đề/Van, Đồ thị Telemetry, Dừng khẩn cấp, Bảng Cài đặt Ngưỡng an toàn & Lưu `localStorage`, Cấu hình Wi-Fi/OTA) **hoạt động 100% mượt mà không cần mạng**. Riêng nút **"🧠 Phân Tích Dữ Liệu Log & Telemetry AI"** cần backend server để gọi API Gemini.
   - **Để bật tính năng Chẩn Đoán AI Gemini trên máy Local**:
     1. Cài đặt **Node.js** trên Windows 11 (nếu chưa có).
     2. Tạo file `.env` tại thư mục gốc project với nội dung: `GEMINI_API_KEY=mã_key_của_bạn` (lấy tại Google AI Studio).
     3. Chạy lệnh `npm start` (hoặc `node server.js`) trong Terminal.
     4. Mở trình duyệt tại địa chỉ `http://localhost:3000` -> Sử dụng đầy đủ tính năng phân tích & chẩn đoán lỗi chuyên sâu bằng Gemini AI!

5. **Lưu ý trên Windows 11 & Khắc phục sự cố**:
   - **Driver USB**: Windows 11 đa số tự động nhận diện chip CH340 / CP2102. Nếu không thấy xuất hiện cổng COM trong danh sách, hãy cài driver `CP210x Universal Windows Driver` hoặc `CH340 Windows Driver`.
   - **Lỗi Cổng COM bị chiếm dụng (`The port is already open`)**: Hãy chắc chắn rằng bạn đã tắt **Serial Monitor** hoặc **Serial Plotter** trong phần mềm Arduino IDE (hoặc các phần mềm terminal như PuTTY) trước khi bấm kết nối trên Web.

---

## Hướng Dẫn Upload Firmware ESP32

### Cách 1: Tự động Biên Dịch & Nạp Qua CI/CD GitHub Actions (Khuyên dùng)
- Mỗi khi push code lên nhánh `main`, GitHub Actions (`.github/workflows/compile-firmware.yml`) sẽ tự động cài đặt `arduino-cli`, biên dịch mã nguồn `ECU_ManualV1.ino` và tạo sẵn file `firmware.bin` tại thư mục `PROJECT_CURRENT/Firmware/ECU_ManualV1/firmware.bin`.
- Bạn chỉ cần tải file `firmware.bin` về và dùng nút **"🔥 Nạp Firmware USB"** trên Web Dashboard để nạp.

### Cách 2: Upload Bằng Arduino IDE Thủ Công
1. Mở file `Firmware/ECU_ManualV1/ECU_ManualV1.ino` bằng **Arduino IDE**.
2. Chọn Board: **ESP32 Dev Module** (hoặc NodeMCU-32S).
3. Thư viện phần cứng cần có trong Arduino IDE:
   - `Adafruit_MAX31855` (đọc cảm biến nhiệt độ EGT qua SPI).
   - `SD` (đọc/ghi cấu hình & log dữ liệu ra thẻ MicroSD).
   - *Lưu ý*: Tín hiệu ESC PWM cho Bơm & Đề được điều khiển bằng bộ tạo xung LEDC phần cứng có sẵn của ESP32 — **không** cần thư viện `ESP32Servo`.
4. Upload Firmware → Baudrate Serial mặc định **115200 baud**.

---

## Tính Năng Chính & Cải Tiến (v8.0)

- **Kiến Trúc Đa Nhân FreeRTOS Cô Lập Tuyệt Đối (Multi-Core Isolation)**:
  - **Core 1 (Engine Control Core)**: Xử lý độc quyền các tác vụ thời gian thực mức phần cứng (RPM ISR, cảm biến EGT SPI, điều xung ESC PWM, bộ lọc RPM 7 tầng, logic ngắt khẩn cấp). Đảm bảo tính thời gian thực 100% không bị trễ do mạng hay thẻ SD.
  - **Core 0 (System & Network Core)**: Đảm nhận việc ghi thẻ SD bất đồng bộ (`sdQueue`), kết nối Wi-Fi, xử lý giao thức Web Serial, quét mạng Wi-Fi không chặn (`WiFi.scanNetworks(true)`) và cập nhật firmware qua mạng Async OTA (`ArduinoOTA`).

- **Cập Nhật Firmware Qua Mạng OTA Bất Đồng Bộ (`ArduinoOTA`)**:
  - Hỗ trợ nạp firmware không dây qua mạng Wi-Fi local mà không cần cắm cáp USB.
  - Tự động ngắt toàn bộ động cơ (`allOff()`) khi bắt đầu nhận bản nạp OTA để đảm bảo an toàn tuyệt đối.

- **Trình Quét Mạng Wi-Fi Không Chặn & Modal Chọn Mạng**:
  - Lệnh `wifi scan` hoạt động ở chế độ Async không gây đơ lag thuật toán tính toán RPM/EGT.
  - Web Dashboard hiển thị danh sách các mạng Wi-Fi quét được, nhấp chuột để chọn SSID tự động và điền mật khẩu.

- **Ghi Log Telemetry & Sự Kiện Lên Thẻ SD (`/ECUxxx.CSV`) Kháng Lỗi C++**:
  - Tự động tạo file nhật ký xoay vòng (`ECU000.CSV` .. `ECU999.CSV`) khi khởi động.
  - Hàng đợi log sự kiện phi đối xứng `sdLogEvent` hỗ trợ hai kiểu tham số `sdLogEvent(const char*)` và `sdLogEvent(const String&)` có bảo vệ null-pointer, đẩy vào queue FreeRTOS truyền dữ liệu thread-safe.

- **Tùy Chỉnh Ngưỡng An Toàn Linh Hoạt (Safety Thresholds Configuration)**:
  - Cho phép người dùng trực tiếp thay đổi và lưu các ngưỡng an toàn trên giao diện Web Dashboard:
    - **PEGT 3s Max (°C)**: Giới hạn dự báo nhiệt độ 3s (mặc định 740°C, tùy chỉnh 300 - 1200°C).
    - **EGT Max (°C)**: Giới hạn nhiệt độ EGT đo trực tiếp (mặc định 800°C, tùy chỉnh 300 - 1200°C).
    - **Max RPM**: Giới hạn vòng tua tối đa chống overrev (mặc định 160.000 RPM, tùy chỉnh 5.000 - 250.000 RPM).
  - Tự động lưu vào **localStorage** của trình duyệt và truyền ngay các lệnh thiết lập (`set pegtmax`, `set egtmax`, `set maxrpm`) tới ECU khi kết nối thành công. Đồng thời được lưu lâu dài vào file cấu hình `/ECUCFG.TXT` trên thẻ SD.

- **Bảo Vệ Chống Overrev & Bùng Nhiệt Toàn Diện**:
  - `ERR:E06`: Tự động ngắt Bơm nhiên liệu và kích hoạt Dừng Khẩn Cấp khi vòng tua lọc `fRpm` vượt ngưỡng `maxRpmLimit`.
  - `ERR:E02`: Tự động ngắt Bơm nhiên liệu và kích hoạt làm mát khi `PEGT 3s > pegtLimit` hoặc `EGT > egtMaxLimit`.

- **Thuật Toán Gia Tốc Mượt Bơm (Pump S-Curve Smoothstep Ramp 1.5s)**: Gia tốc ga nhiên liệu mượt mà theo đa thức bậc 3 ($3x^2 - 2x^3$), giúp động cơ rú ga mượt hệt như máy bay thật, chống sặc dầu & dập lửa.

- **Thuật Toán Gia Tốc Đề (Starter Exponential Ramp Up 1.0s)**: Gia tốc lực kéo mô-tơ Đề theo đường cong mũ lũy thừa, thắng lực ma sát nghỉ ban đầu mà không làm xóc nhông Bendix hay kẹt motor.

- **Bộ Lọc RPM 7 Tầng Chuyên Sâu**: Min Pulse → Dynamic Mask 75% → Median-5 → Dynamic Outlier Guard → Rate Limiter → Adaptive PWM-Aware Learning (Monotonicity Floor + Hysteresis) → Cascaded Dual-EMA Bậc 2.

- **Dừng Khẩn Cấp Thổi Khí Nóng (`estop`)**: Ngắt Bơm/Lửa/Van nhưng tự động bật/giữ Mô-tơ Đề 1300 µs để làm mát buồng đốt khi EGT > 80°C.

- **Nút Cứng Vật Lý Đa Năng (`BTN1` / IO22)**:
  - Nhấp 1 lần (< 3s): Kích hoạt Dừng Khẩn Cấp làm mát.
  - Nhấn giữ 3s (>= 3s): Dừng Toàn Bộ (Full Shutdown, ngắt cả Đề).

- **Khóa An Toàn 2 Chiều (Pump-Valve Interlock)**:
  - Không cho bật Bơm khi cả 2 van nhiên liệu đang ĐÓNG (`ERR:E01`).
  - Tự động ngắt Bơm khi cả 2 van bị đóng (`EV:A02`).
  - Tự động đóng cả 2 van khi tắt Bơm (`EV:A01`).

- **Giao Thức Bảo Mật CRC-8 & Web Serial API**: Loại bỏ 100% lệnh rác do nhiễu cao tần từ ESC/Motor.

---

## Bảng Mã Sự Kiện & Mã Lỗi (Event & Error Code Dictionary)

| Mã Serial | Loại | Ý Nghĩa | Thông Báo Tiếng Việt Hiển Thị Trên Web Dashboard |
|-----------|------|---------|--------------------------------------------------|
| `ERR:E01` | Cảnh báo | Bật Bơm khi cả 2 van đóng | `⚠️ CẢNH BÁO AN TOÀN: Cả 2 van nhiên liệu đều đang ĐÓNG! Vui lòng mở ít nhất 1 van trước.` |
| `ERR:E02` | Khẩn cấp | Quá nhiệt PEGT / EGT vượt ngưỡng | `🛑 CẢNH BẢO QUÁ NHIỆT: Nhiệt độ EGT/PEGT vượt ngưỡng an toàn! Đã tự động NGẮT BƠM & THỔI LÀM MÁT.` |
| `ERR:E06` | Khẩn cấp | Quá vòng tua RPM > Max RPM | `🛑 CẢNH BẢO QUÁ VÒNG TUA: Vòng tua (RPM) vượt ngưỡng tối đa! Đã tự động NGẮT BƠM NHIÊN LIỆU.` |
| `EV:A01`  | Thông tin | Tắt Bơm -> Tự đóng 2 van | `ℹ️ TỰ ĐỘNG KHÓA: Đã Tắt Bơm -> Tự động đóng cả 2 Van nhiên liệu (V1 & V2).` |
| `EV:A02`  | Thông tin | Đóng 2 van -> Tự ngắt Bơm | `ℹ️ TỰ ĐỘNG KHÓA: Đã đóng cả 2 Van -> Tự động ngắt Bơm nhiên liệu.` |
| `EV:A03`  | Khẩn cấp | Dừng Khẩn Cấp -> Thổi khí 1300µs | `🛑 DỪNG KHẨN CẤP: Ngắt Bơm & Lửa/Van -> Tự động bật Đề 1300µs thổi khí làm mát.` |
| `EV:A04`  | Thông tin | Dừng Khẩn Cấp -> Máy nguội <= 80°C | `ℹ️ DỪNG KHẨN CẤP: Đã ngắt Bơm & Lửa/Van (Máy đã nguội <= 80°C).` |
| `EV:A05`  | Khẩn cấp | Nút 3s / Dừng Toàn Bộ | `⚡ DỪNG TOÀN BỘ: Đã ngắt 100% tất cả thiết bị (Bơm, Đề, Lửa, Van).` |

---

## Pinout ESP32

| Chức năng | GPIO | Ghép nối / Chú thích |
|-----------|------|----------------------|
| MAX31855 CLK/CS/DO | 18 / 5 / 19 | SPI (VSPI) |
| RPM Sensor | **33** | Cảm biến từ/quang (KMZ10A / Hall) |
| Pump ESC | 26 (LEDC CH0) | Tín hiệu PWM ESC Bơm nhiên liệu |
| Starter ESC | 25 (LEDC CH1) | Tín hiệu PWM ESC Mô-tơ Đề |
| Valve 1 / Valve 2 | 17 / 16 | Tín hiệu Digital điều khiển van |
| Ignition/Glow | 32 | Tín hiệu Digital điều khiển đánh lửa |
| SD CS/SCK/MOSI/MISO | 13 / 14 / 23 / 27 | Thẻ nhớ MicroSD (HSPI) |
| User Button (BTN1) | 22 | Nút nhấn cứng đa năng (Active LOW) |
| Status LED | 2 | Nháy Heartbeat 1Hz |

---

## Cảnh Báo An Toàn

- ESP32 chỉ chịu **3.3V** — dùng level converter cho tín hiệu 5V
- Chỉ dùng **thermocouple loại K**
- Cảm biến RPM KMZ10A cần hiệu chỉnh trimpot **RP1/RP2/RP3** trước khi dùng
- ESP32 cần nguồn điện riêng (không cấp qua USB khi motor chạy)


