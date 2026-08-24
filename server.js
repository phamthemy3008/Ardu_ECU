import 'dotenv/config';
import express from 'express';
import path from 'path';
import fs from 'fs';
import { fileURLToPath } from 'url';
import { GoogleGenAI, ThinkingLevel } from '@google/genai';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

const app = express();
const PORT = process.env.PORT || 3000;

app.use(express.json({ limit: '2mb' }));

app.get('/api/health', (req, res) => {
  res.json({ status: 'ok', app: 'Ardu ECU Dashboard' });
});

let aiClient = null;
function getGeminiClient() {
  if (!aiClient) {
    const apiKey = process.env.GEMINI_API_KEY;
    if (!apiKey) {
      throw new Error('GEMINI_API_KEY environment variable is missing.');
    }
    aiClient = new GoogleGenAI({
      apiKey: apiKey,
      httpOptions: {
        headers: {
          'User-Agent': 'aistudio-build',
        },
      },
    });
  }
  return aiClient;
}

app.post('/api/analyze-logs', async (req, res) => {
  try {
    const { logs, errors, telemetryContext, deepAnalysis } = req.body;
    
    if (!logs && !errors) {
      return res.status(400).json({ error: 'Không tìm thấy dữ liệu nhật ký (logs) để phân tích.' });
    }

    const ai = getGeminiClient();

    const systemInstruction = `Bạn là Chuyên gia Cao cấp về Hệ thống Điều khiển Động cơ Phản lực Mô hình (Jet Engine ECU & Controls Engineer) cho hệ thống Ardu ECU (Firmware v5.9 ESP32).

Nhiệm vụ của bạn là phân tích dữ liệu Telemetry, Nhật ký Console và các Mã Lỗi (ERR) để đưa ra báo cáo chẩn đoán chuyên sâu và hướng dẫn kỹ thuật chi tiết theo các phần sau:

1. 🛑 **TỔNG QUAN & ĐÁNH GIÁ MỨC ĐỘ RỦI RO**:
   - Đánh giá mức rủi ro (AN TOÀN / CẦN LƯU Ý / NGUY HIỂM CAO).
   - Tóm tắt trạng thái động cơ hiện tại (Chạy mồi, Đang đề, Đang rú ga, Dừng khẩn cấp, hay Bị quá nhiệt/quá tua).

2. 🔍 **PHÂN TÍCH MÃ LỖI (ERR) & SỰ KIỆN (EV)**:
   - Giải thích nguyên nhân gốc rễ (Root Cause) của các mã lỗi nếu có:
     * ERR:E01 (Mất/Lỗi tín hiệu RPM hoặc mở Bơm khi 2 van đóng)
     * ERR:E02 (Quá nhiệt PEGT 3s dự báo > pegtMax hoặc EGT > egtMax)
     * ERR:E06 (Quá vòng tua fRpm > maxRpm)
     * EV:A01/EV:A02 (Khóa liên động Bơm - Van nhiên liệu)
     * EV:A03 (Dừng khẩn cấp Dội khí làm mát 1300µs)
     * Lỗi thẻ nhớ SD hoặc mất kết nối UART.

3. 📊 **PHÂN TÍCH NHIỄU TÍN HIỆU & CHỈ SỐ DSP (RPM & EGT)**:
   - So sánh vòng tua thô RRPM và vòng tua lọc FRPM / RPM hiển thị. Nếu RRPM nhảy vọt tột đột ngột hoặc xuất hiện nhiễu chùm (burst noise), nhận diện nguy cơ nhiễu cao tần (EMI) từ ESC Motor Brushless truyền qua đường UART hoặc cảm biến.
   - Nhận diện các bất thường về tốc độ tăng nhiệt dEGT (°C/s) và dự báo PEGT 3s.

4. 🛠️ **HƯỚNG DẪN TINH CHỈNH CODE FIRMWARE ('ECU_ManualV1.ino') CHI TIẾT**:
   - Đưa ra các kiến nghị điều chỉnh hằng số / vị trí code trong 'ECU_ManualV1.ino' cụ thể:
     * **Nếu bị nhiễu RPM (Spikes/Burst Noise)**:
       - Tăng 'MIN_PULSE_US' (mặc định 330µs) trong hàm 'calculateRpm()' để chặn xung rác tần số cao.
       - Tăng kích thước bộ lọc trung vị 'MEDIAN_FILTER_SIZE' (mặc định 5 phần tử) hoặc siết mặt nạ động 'DYNAMIC_MASK_RATIO' (mặc định 0.75).
       - Giảm hệ số làm mượt 'EMA_ALPHA_RUNNING' (mặc định 0.25 xuống 0.15) để triệt nhấp nháy đồ thị.
       - Siết giới hạn gia tốc vòng tua 'SLEW_RATE_MAX_RPM_PER_SEC'.
     * **Nếu bị bùng nhiệt Hot-Start / Trễ ga / Quá nhiệt**:
       - Điều chỉnh thời gian ramp bơm 'PUMP_RAMP_DURATION_MS' (mặc định 1500ms Smoothstep S-Curve) hoặc bật/tắt qua lệnh 'set pumpramp on|off'.
       - Điều chỉnh thời gian ramp đề 'STARTER_RAMP_DURATION_MS' (mặc định 1000ms Exponential Ramp).
       - Tùy chỉnh hằng số dự báo 'PEGT_LOOKAHEAD_SEC' (3.0s) hoặc truyền lệnh điều chỉnh ngưỡng trên Web UI: 'set pegtmax <giá_trị>', 'set egtmax <giá_trị>', 'set maxrpm <giá_trị>'.

5. ⚡ **BẢO TRÌ & BỎ CHỐNG NHIỄU PHẦN CỨNG (HARDWARE CHECKLIST)**:
   - Kiểm tra tiếp địa chung (Common Ground) giữa ESP32, ESC Bơm, ESC Đề và Pin LiPo.
   - Thêm tụ gốm 0.1µF song song với cảm biến RPM (KMZ10A/Hall) hoặc căn chỉnh lại khoảng cách nam châm/đĩa quang.
   - Sử dụng cáp xoắn chống nhiễu (Shielded Twisted Pair) cho dây SPI module MAX31855.

Hãy trình bày báo cáo bằng tiếng Việt mạch lạc, chuyên nghiệp, sử dụng Markdown sinh động với biểu tượng cảm xúc (emoji) trực quan.`;

    const prompt = `Dưới đây là dữ liệu telemetry, mã lỗi và toàn bộ nhật ký console từ Ardu ECU (Firmware v5.9):

--- DỮ LIỆU CẢNH BÁO / MÃ LỖI (ERR & EV) ---
${errors && errors.length > 0 ? errors.join('\n') : 'Không ghi nhận mã lỗi ERR nào trong phiên làm việc hiện tại.'}

--- TRẠNG THÁI TELEMETRY HIỆN TẠI ---
${telemetryContext || 'Chưa có telemetry'}

--- MẪU LOG CONSOLE GẦN ĐÂY ---
${logs || 'Không có dữ liệu log'}

Hãy tiến hành phân tích toàn diện dữ liệu trên và đưa ra báo cáo chẩn đoán chi tiết kèm hướng dẫn tinh chỉnh code/phần cứng cho kỹ sư vận hành.`;

    let modelName = 'gemini-3.6-flash';
    let configObj = {
      systemInstruction,
      temperature: 0.3,
    };

    if (deepAnalysis) {
      modelName = 'gemini-2.5-flash';
      configObj = {
        systemInstruction,
        temperature: 0.2,
      };
    }

    let response;
    try {
      response = await ai.models.generateContent({
        model: modelName,
        contents: prompt,
        config: configObj,
      });
    } catch (modelErr) {
      console.warn(`Primary model ${modelName} failed (${modelErr.message}), falling back to gemini-3.6-flash...`);
      modelName = 'gemini-3.6-flash';
      response = await ai.models.generateContent({
        model: 'gemini-3.6-flash',
        contents: prompt,
        config: {
          systemInstruction,
          temperature: 0.3,
        },
      });
    }

    res.json({ analysis: response.text, modelUsed: modelName });
  } catch (err) {
    console.error('Error analyzing ECU logs:', err);
    res.status(500).json({
      error: 'Không thể phân tích dữ liệu log bằng AI. Chi tiết: ' + (err.message || 'Lỗi kết nối Gemini API'),
    });
  }
});

const toolsDir = path.join(__dirname, 'PROJECT_CURRENT', 'Tools');
const firmwareDir = path.join(__dirname, 'PROJECT_CURRENT', 'Firmware', 'ECU_ManualV1');
const versionPath = path.join(__dirname, 'version.json');

function getAppVersion() {
  try {
    if (fs.existsSync(versionPath)) {
      const vData = JSON.parse(fs.readFileSync(versionPath, 'utf8'));
      return vData.version || '6.4';
    }
  } catch (e) {
    console.error('Error reading version.json:', e);
  }
  return '6.4';
}

// API Lấy thông tin Version hiện tại của dự án
app.get('/api/version', (req, res) => {
  res.setHeader('Cache-Control', 'no-store, no-cache, must-revalidate, proxy-revalidate');
  const v = getAppVersion();
  res.json({
    version: v,
    webUiVersion: `Web UI v${v}`,
    firmwareVersion: `VER=${v}`,
    updatedAt: new Date().toISOString()
  });
});

// API Thông tin Firmware
app.get('/api/firmware/info', (req, res) => {
  try {
    res.setHeader('Cache-Control', 'no-store, no-cache, must-revalidate, proxy-revalidate');
    const inoPath = path.join(firmwareDir, 'ECU_ManualV1.ino');
    let fileSize = 0;
    if (fs.existsSync(inoPath)) {
      const stats = fs.statSync(inoPath);
      fileSize = stats.size;
    }
    const currentVer = getAppVersion();
    res.json({
      name: 'Ardu ECU Manual V1 Firmware',
      version: `v${currentVer} ESP32`,
      rawVersion: currentVer,
      filename: 'ECU_ManualV1.ino',
      size: fileSize,
      updatedAt: new Date().toISOString(),
      downloadUrl: '/api/firmware/download?type=ino'
    });
  } catch (err) {
    res.status(500).json({ error: 'Không thể lấy thông tin Firmware' });
  }
});

// API Tải về Firmware (.ino)
app.get('/api/firmware/download', (req, res) => {
  const inoPath = path.join(firmwareDir, 'ECU_ManualV1.ino');
  if (fs.existsSync(inoPath)) {
    res.setHeader('Content-Type', 'text/plain; charset=utf-8');
    res.setHeader('Content-Disposition', 'attachment; filename="ECU_ManualV1.ino"');
    res.sendFile(inoPath);
  } else {
    res.status(404).json({ error: 'Không tìm thấy file Firmware ECU_ManualV1.ino' });
  }
});

// Endpoint phục vụ File Binary OTA (.bin) cho ESP32 HTTP OTA
app.get(['/firmware.bin', '/api/firmware/download-bin'], (req, res) => {
  const binPathPrimary = path.join(firmwareDir, 'firmware.bin');
  const binPathSecondary = path.join(firmwareDir, 'ECU_ManualV1.bin');
  
  let targetBinPath = null;
  if (fs.existsSync(binPathPrimary)) {
    targetBinPath = binPathPrimary;
  } else if (fs.existsSync(binPathSecondary)) {
    targetBinPath = binPathSecondary;
  }

  if (targetBinPath) {
    res.setHeader('Content-Type', 'application/octet-stream');
    res.setHeader('Content-Disposition', 'attachment; filename="firmware.bin"');
    res.sendFile(targetBinPath);
  } else {
    res.status(404).send('Chưa có file compiled firmware.bin trong thư mục Firmware.');
  }
});

// Endpoint upload file firmware.bin trực tiếp từ Web Dashboard
app.post('/api/firmware/upload-bin', express.raw({ type: '*/*', limit: '10mb' }), (req, res) => {
  try {
    if (!req.body || req.body.length === 0) {
      return res.status(400).json({ error: 'Nội dung file rỗng hoặc không đúng định dạng.' });
    }
    const binPath = path.join(firmwareDir, 'firmware.bin');
    fs.writeFileSync(binPath, req.body);
    console.log(`[Upload OTA] Đã lưu file firmware.bin thành công (${req.body.length} bytes)`);
    return res.json({ success: true, message: 'Đã tải file firmware.bin lên Server thành công!', size: req.body.length });
  } catch (err) {
    console.error('[Upload OTA Error]', err);
    return res.status(500).json({ error: 'Lỗi khi lưu file firmware.bin trên Server' });
  }
});


// API Danh sách và tải về 3D STL
app.get("/api/stl/list", (req, res) => {
  try {
    const stlDir = path.join(__dirname, "REFERENCES", "Hardware", "3D Printable Files");
    if (!fs.existsSync(stlDir)) return res.json({ files: [] });
    const files = fs.readdirSync(stlDir).filter(f => f.endsWith(".stl")).map(f => {
      const stats = fs.statSync(path.join(stlDir, f));
      return { filename: f, size: stats.size, downloadUrl: `/api/stl/download/${encodeURIComponent(f)}` };
    });
    res.json({ files });
  } catch(e) {
    res.status(500).json({ error: e.message });
  }
});

app.get("/api/stl/download/:filename", (req, res) => {
  const filename = req.params.filename;
  const stlPath = path.join(__dirname, "REFERENCES", "Hardware", "3D Printable Files", filename);
  if (fs.existsSync(stlPath)) {
    res.setHeader("Content-Type", "application/octet-stream");
    res.setHeader("Content-Disposition", `attachment; filename="${filename}"`);
    res.sendFile(stlPath);
  } else {
    res.status(404).json({ error: "Không tìm thấy file STL" });
  }
});

app.use(express.static(toolsDir, {
  etag: false,
  setHeaders: (res, filePath) => {
    if (filePath.endsWith('.html') || filePath.endsWith('.json')) {
      res.setHeader('Cache-Control', 'no-store, no-cache, must-revalidate, proxy-revalidate');
      res.setHeader('Pragma', 'no-cache');
      res.setHeader('Expires', '0');
    }
  }
}));

app.get('*', (req, res) => {
  res.setHeader('Cache-Control', 'no-store, no-cache, must-revalidate, proxy-revalidate');
  res.setHeader('Pragma', 'no-cache');
  res.setHeader('Expires', '0');
  res.sendFile(path.join(toolsDir, 'index.html'));
});

app.listen(PORT, '0.0.0.0', () => {
  console.log(`Ardu ECU Dashboard server running on http://0.0.0.0:${PORT}`);
});


