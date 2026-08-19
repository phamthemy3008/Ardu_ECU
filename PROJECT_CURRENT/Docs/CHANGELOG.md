# Ardu ECU Manual V1 - Changelog & Version History

## [v7.8] - 2026-08-10
- **CI/CD Auto Release**: Automatically incremented version to v7.8.
- Synchronized Web UI v7.8 and Firmware ECU_ManualV1.ino (VER=7.8).

## [v7.7] - 2026-08-10
- **CI/CD Auto Release**: Automatically incremented version to v7.7.
- Synchronized Web UI v7.7 and Firmware ECU_ManualV1.ino (VER=7.7).

## [v7.6] - 2026-08-09
- **Sửa lỗi Linker C++ `sdLogEvent`**: Bổ sung hàm overload `sdLogEvent(const char* msg)` có kiểm tra con trỏ `NULL` và đẩy vào hàng đợi FreeRTOS thread-safe song song với `sdLogEvent(const String& msg)`. Khắc phục triệt để lỗi `undefined reference to _Z10sdLogEventPKc` khi biên dịch firmware ESP32.
- **Tối ưu CI/CD GitHub Actions (`compile-firmware.yml`)**:
  - Khai báo quyền `permissions: contents: write` để cấp quyền ghi repository cho `GITHUB_TOKEN`.
  - Tích hợp bước `git pull --rebase origin main` tự động giải quyết xung đột commit giữa runner và remote repo trước khi push file `firmware.bin`.
- **Đồng bộ hóa tài liệu & v7.6**: Cập nhật toàn bộ tài liệu hướng dẫn sử dụng (`README.md`), thông số kỹ thuật và hướng dẫn nạp firmware/chạy Web Dashboard.

## [v6.9] - 2026-08-07
- **Non-blocking Wi-Fi Scanner (ESP32)**: Implemented `wifi scan` command using ESP32 `WiFi.scanNetworks(true)` async mode so background network scanning won't interrupt Core 1 engine control timing.
- **Web Dashboard Wi-Fi Network Picker**: Added `🔍 Quét Wi-Fi` button and interactive scanned Wi-Fi network list in the Wi-Fi modal (`Tools/index.html`). Clicking any scanned SSID automatically fills the SSID field and focuses password entry.
- Synchronized Web UI v6.9 and Firmware ECU_ManualV1.ino (VER=6.9).

## [v6.8] - 2026-08-07
- **Wi-Fi & OTA Configuration System**: Added full Wi-Fi network configuration saved in ESP32 Flash NVS and SD card (`/ECUCFG.TXT`).
- **Auto-Connect & Auto-OTA Init**: ESP32 connects to configured local Wi-Fi router on boot and automatically initializes ArduinoOTA on Core 0 as soon as Wi-Fi establishes connection.
- **Web Dashboard Wi-Fi Modal**: Added `📶 Cấu Hình Wi-Fi & OTA` modal button and live `📶 Wi-Fi: <IP / OFF>` badge to configure SSID, Password, and OTA commands (`set wifi <ssid> <pass>`).

## [v6.7] - 2026-08-07
- **Async OTA Update Support (Core 0)**: Implemented non-blocking Over-The-Air (ArduinoOTA) firmware flashing pinned exclusively to Core 0 (`otaTaskWorker`).
- **Core 1 Hardening & Interlock**: Core 1 main engine control loop remains 100% deterministic (0% CPU impact from OTA network/flash processing). Added automatic safety cut-off (`allOff()`) when OTA flash begins.
- **Web UI Monitoring**: Added real-time `📶 OTA` status badge on Web Dashboard header and serial command handlers (`set ota on/off`).

## [v6.6] - 2026-08-07
- **CI/CD Auto Release**: Automatically incremented version to v6.6.
- Synchronized Web UI v6.6 and Firmware ECU_ManualV1.ino (VER=6.6).

## [v6.5] - 2026-08-07
- **CI/CD Auto Release**: Automatically incremented version to v6.5.
- Synchronized Web UI v6.5 and Firmware ECU_ManualV1.ino (VER=6.5).

## [v6.4] - 2026-08-06
- **Core Allocation Optimization**: Moved SD Logging to dedicated FreeRTOS async task pinned to Core 0 with thread-safe mutex and lockless queue.
- **Core 1 Isolation**: Reserved Core 1 exclusively for high-precision ECU control loops (RPM ISR, EGT sampling, PWM/ESC outputs).
- **Web UI & Dashboard**: Updated Web UI v6.4 version display and real-time Telemetry monitoring.

## [v6.3] - 2026-08-06
- Initial release of Web UI v6.3 with high-frequency Telemetry and AI Log Analysis.
