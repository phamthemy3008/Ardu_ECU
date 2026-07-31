# Quy Trình Khởi Động (RPM-gated ignition)

Tài liệu mô tả trình tự khởi động động cơ đã triển khai trong
`ECU_TestV1_EGT_DRY_START_PATCH.ino`. **Mọi giá trị (xung, tốc độ, thời gian,
ngưỡng)** đều chỉnh được trong **Web UI → Settings** và **lưu vào thẻ SD**
(`/ECUCFG.TXT`, nút "Lưu config vào SD"), tự nạp lại khi bật máy.

## Giai đoạn 1 — PURGE
PURGE gồm **3 bước** (dùng chung hàm ramp cho cả dry test lẫn chạy thật):
1. **ESC arm-hold**: giữ starter ở **ESC_SAFE (1000µs, thật sự = 0)** trong
   **2s (`escarmholdms`)** đầu tiên để ESC hoàn tất tự chuỗi "arm" của nó trước khi
   nhận bất kỳ xung khác 0 nào — nếu nhảy thẳng lên 1150µs ngay lập tức, ESC có thể
   mới arm được một phần, biểu hiện ra ngoài giống như "quay vài vòng rồi khựng".
2. Giữ nguyên **1150µs (`rampfromus`)** trong **10s (`purgestablems`)** để chạy ổn định.
3. **Sau đó mới tăng xung +1µs mỗi 250ms (≈ 4µs/s = 2µs/0.5s)**, tới khi **RPM > 3000**
   thì **giữ 3s** để thổi sạch khí/dầu dư, rồi sang giai đoạn kế.

Chưa glow, chưa nhiên liệu. Nếu quá `spinuptimeoutms` (57s, đã tính cả 2s arm-hold +
10s ổn định) mà RPM chưa đạt → abort `NO_RPM`. Điều kiện abort do thiếu RPM starter
(`STARTER_PROVE_TIMEOUT_MS`, 1.5s) chỉ bắt đầu tính **sau khi hết `escarmholdms`**
(RPM = 0 trong lúc arm-hold là bình thường, không phải lỗi starter).

## Giai đoạn 2 — SPINUP_PREHEAT
**Bật glow plug 2s** để làm nóng, sau đó **vẫn giữ glow bật** và sang giai đoạn kế.
Starter chuyển sang chế độ trợ lực theo RPM (xem "ánh xạ starter").

## Giai đoạn 3 — INTRO_FUEL
Glow đang bật. **Bơm nhiên liệu ở mức min (1015µs)** + **mở Start Valve (Valve 1)**.
Sang giai đoạn kế (LIGHTOFF).

## Giai đoạn 4 — LIGHTOFF (mồi lửa + sang van)
- Chờ **~1s (fuel delay)** để nhiên liệu tới buồng đốt.
- **Xác nhận bắt lửa THẬT** (tránh false light-off do glow làm nóng cặp nhiệt mà chưa
  cháy): **EGT ≥ 100°C VÀ dEGT/dt ≥ `lightoffrise` (mặc định 15°C/s), giữ liên tục
  ≥ `lightoffconfirmms` (0.7s)**. → Mở **Main Valve (Valve 2)**.
- Chờ **2s** (`postIgnitionHeatMs`) cho lửa ổn định → **đóng Start Valve (Valve 1)**.
- Chờ thêm **2s** (`flameProveMs`) xem lửa còn cháy (không mất lửa) → **tắt glow**,
  đánh dấu **khởi động thành công**, sang ACCEL_TO_IDLE.
- **Thất bại**: nếu quá **6s** (`noIgnitionTimeoutMs`) mà chưa xác nhận bắt lửa, hoặc
  **mất lửa** trong lúc sang van → tắt glow + bơm + van, đánh dấu **thất bại**, và giữ
  starter chạy (blow-out) trong `purgeoutms` để thổi hết nhiên liệu chưa cháy, rồi cooldown.

## Giai đoạn 5 — ACCEL_TO_IDLE
Chỉ Main Valve (Valve 2), glow tắt. Nếu lửa còn cháy mà **RPM không tăng** thì
**tăng xung bơm +1µs, chờ 5s** quan sát RPM, lặp cho tới khi đạt **idle RPM (42000)**.
(Bước/thời gian là giá trị test, sẽ tăng nhanh hơn khi tìm được thông số phù hợp.)

## Ánh xạ starter theo RPM (trợ lực)
Từ SPINUP_PREHEAT trở đi, xung starter bám theo RPM:
```
startUs = assistMin + (assistMax − assistMin) × (rpm / releaseRpm)   [kẹp assistMin..assistMax]
rpm ≥ releaseRpm (10000)  →  starter OFF (nhả cánh quạt)
```
- `assistMin` = `Assist us` (≈1150µs) — xung khi RPM thấp
- `assistMax` = `Ramp to us` (**1450µs**) — xung tối đa
- `releaseRpm` = `Starter max RPM` (**10000**) — điểm nhả starter, tránh quá tải motor đề

## Bảo vệ (chạy mọi lúc, mọi mode)
1. **Chống quá tải starter**: RPM > **10000** → starter **tắt** (nhả cánh quạt, để RPM tự
   tăng lên idle).
2. **Chống soak-back nhiệt**: nếu **EGT > 90°C** mà **RPM < 3000** → starter tự chạy,
   **bắt đầu 1200µs, tăng +1µs/0.1s** (không giật từ 1150µs) tới khi RPM > 3000 rồi giữ,
   cho tới khi **EGT < 90°C** thì mới dừng — tránh khí nóng dội ngược lên đầu máy khi
   động cơ nóng bị dừng/tụt tua.
3. **Quá nhiệt**: EGT ≥ maxEgt (680°C, có look-ahead) → abort ngay (áp dụng cả khi start).

## Tiêu chí mất lửa (flame-out)
**EGT tụt > 10°C trong ~2s HOẶC EGT < 100°C** (hoặc cặp nhiệt lỗi) → coi là mất lửa.

## Tham số tune (Web UI → Settings → "Tune — Start sequence")
`rampfromus`, `ramptous` (assist max), `rampstepms`, `escarmholdms`, `purgestablems`, `ignarmrpm`, `spinuptimeoutms`,
`fueldelayms`, `lightoffrise`, `lightoffconfirmms`, `flameprovems`, `flameoutdropc`,
`accelholdms`, `accelstepus`, `purgeoutms`; và "Bảo vệ động cơ/starter":
`startermaxrpm`, `hotspinminrpm`, `hotspinus`, `cooltarget` (90°C).

> ⚠️ **Chưa kiểm chứng trên phần cứng.** Cần bench-test toàn bộ trình tự + các cơ cấu
> bảo vệ, và **đo lại bảng lưu lượng bơm `kPumpMap`** dưới xung LEDC trước khi start thật.
