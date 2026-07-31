# Hardware Directory

Thư mục chứa tất cả các tệp liên quan đến phần cứng của dự án Ardu_ECU.

## 📂 Sub-directories

### **Schematics/** - Sơ đồ mạch điện
Chứa tất cả các tệp sơ đồ và thiết kế mạch điện.

**Files**:
- `ECU Rev9-10 Schematic.pdf` - Schema cho Rev 9 và 10
- `Schematic_Ardu ECU_Rev 11.pdf` - Schema cho Rev 11 (ổn định hiện tại)
*(Lưu ý: Các file schema mới nhất nằm trong thư mục `../../PROJECT_CURRENT/Hardware/`)*

### **Components/** - Linh kiện và tài liệu
Chứa hình ảnh và thông số kỹ thuật của các linh kiện được sử dụng.

**Components**:
- `5v-to-3-3v-logic-level-converter-500x500.jpg` - Logic level converter
- `CK1602 Power Supply.jpg` - Power supply unit
- `ESP32 Dev Kit.jpeg` - Main microcontroller (NodeMCU 32S)
- `LR-7843.jpg` - Additional component

### **3D Printable Files/** - Các file in 3D
Chứa các file STL cho các bộ phận cơ học có thể in 3D.

**Files**:
- `Bendix Sleeve.stl` - Sleeves for starter motor
- `Brushed motor mount.stl` - Mount cho động cơ brushed
- `Brushless motor mount.stl` - Mount cho động cơ brushless

## 🔧 Hardware Overview

### **Core Components**:
1. **Microcontroller**: ESP32 DevKit V1 (NodeMCU-32S)
2. **Thermocouple Reader**: MAX31855 (for EGT measurement)
3. **SD Card Module**: For data logging
4. **Power Supply**: CK1602 unit
5. **Logic Level Converter**: 5V to 3.3V conversion
6. **Control Outputs**:
   - Starter motor controller (brushed or brushless)
   - Fuel pump controller (ESC or similar)
   - Ignition control (glow plug driver)
   - Gas valve solenoid control (MOSFET module)

### **Sensors**:
- **EGT Sensor**: K-type Thermocouple with MAX31855
- **RPM Sensor**: Magnetic (Hall effect) or Optical sensor
- **Optional**: Pressure sensors, Load cells

## 📐 Design Files

### **Latest Reference Files** (2026-07-23):
- `../../PROJECT_CURRENT/Hardware/ECU_JET_20260723.net` - Netlist mạch ECU mới nhất (gồm cả RPM sensor)
- `../../PROJECT_CURRENT/Hardware/SCH_MinijetengineECU_20260723.json` - Current ECU design schema

These are reference files from the current system implementation (ManualV1 v5.3) and can be used as guidelines for future designs.

## 🚀 Assembly Notes

1. **PCB Layout**: Refer to schematics for component placement
2. **3D Printed Parts**: Use `.stl` files for 3D printing
3. **Component Mounting**: Follow schematic for orientation and pin connections
4. **Wiring**: Carefully follow schematic connections to avoid damage

## ⚠️ Important Warnings

- **Voltage**: ESP32 operates at 3.3V - use logic level converter for 5V devices
- **Power**: Use proper power supply rated for your application
- **Thermocouple**: K-type only - do not use other types
- **High Temperature**: Mount components away from engine exhaust

## 📊 Revision History

| Revision | Date | Key Features |
|----------|------|--------------|
| Rev 9 | Early | Initial design |
| Rev 10 | - | WiFi control, OLED display |
| Rev 11 | - | SD Card, dual-core processing, major redesign |
| Rev 12 TC10 | Legacy | Multi-sensor support, Kero start |
| TestV1_EGT_DRY_START | Legacy | Firmware cũ, có Auto-start, WiFi |
| ManualV1 (v5.3) | Current | Bản hiện tại, Manual Serial Control, Compact Event Codes, Interlock & Toast Banners |

---

For detailed firmware information, see [Firmware/README.md](../Firmware/README.md)

For project overview, see [README.md](../README.md)
