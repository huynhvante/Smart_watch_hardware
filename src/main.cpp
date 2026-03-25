#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <NimBLEDevice.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ── ESP32-C3: bật USB CDC để Serial hoạt động qua USB ────────────────────────
// Nếu dùng Arduino IDE: Tools → USB CDC On Boot → Enabled
// Nếu dùng PlatformIO, thêm vào platformio.ini:
//   build_flags = -DARDUINO_USB_CDC_ON_BOOT=1
//                 -DARDUINO_USB_MODE=1
#ifndef ARDUINO_USB_CDC_ON_BOOT
  #define ARDUINO_USB_CDC_ON_BOOT 0
#endif

// ── UUID ──────────────────────────────────────────────────────────────────────
#define SERVICE_UUID      "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_MPU_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // notify batch magnitude
#define CHAR_HEALTH_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26aa"  // notify BPM/SpO2 mỗi 1s
#define CHAR_COMMAND_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"  // write (command + datetime)
// ── UUID mới cho datetime ─────────────────────────────────────────────────────
// Phone ghi chuỗi "YYYY-MM-DD HH:MM:SS" vào characteristic này
#define CHAR_DATETIME_UUID "beb5483e-36e1-4688-b7f5-ea07361b26ab"  // write datetime

// ── Pin & OLED ────────────────────────────────────────────────────────────────
#define I2C_SDA   8
#define I2C_SCL   9
#define OLED_ADDR 0x3C
#define OLED_W    128
#define OLED_H    64
#define OLED_RST  -1

// ── MPU6050 ───────────────────────────────────────────────────────────────────
#define MPU_ADDR        0x68
#define MPU_PWR_MGMT    0x6B
#define MPU_ACCEL_XOUT  0x3B
#define MPU_INTERVAL_MS 15
#define MPU_BATCH_SIZE  8

// ── MAX30102 ──────────────────────────────────────────────────────────────────
#define BUF_SIZE              100
#define SHIFT_SIZE            25
#define AGC_DC_LOW            20000UL
#define AGC_DC_HIGH           50000UL
#define AGC_STEP              5
#define AGC_UPDATE_N          25
#define AGC_BR_MIN            10
#define AGC_BR_MAX            255
#define FINGER_THR            10000UL
#define FIR_LEN               5
#define BPM_MIN               40
#define BPM_MAX               200
#define SPO2_MIN              85
#define SPO2_MAX              100
#define MED_LEN               5
#define HEALTH_BLE_INTERVAL_MS 1000UL

// ── Fall Detection ────────────────────────────────────────────────────────────
// Ngưỡng: mag < FREE_FALL_THR → rơi tự do, sau đó mag > IMPACT_THR → va chạm
#define FREE_FALL_THR   0.4f    // g
#define IMPACT_THR      2.5f    // g
#define FALL_WINDOW_MS  500     // thời gian tối đa giữa freefall→impact
#define FALL_ALERT_MS   5000    // hiển thị cảnh báo FALL 5 giây

static const float FIR_W[FIR_LEN] = { 0.10f, 0.20f, 0.40f, 0.20f, 0.10f };

struct Biquad {
    float b0,b1,b2,a1,a2,z1,z2;
    Biquad(float b0,float b1,float b2,float a1,float a2)
        : b0(b0),b1(b1),b2(b2),a1(a1),a2(a2),z1(0),z2(0) {}
};
struct FIRState  { float buf[FIR_LEN] = {}; int head = 0; };
struct AGCState  { uint8_t br = 60; uint32_t accum = 0; int n = 0; };
struct MedianBuf { int32_t arr[MED_LEN] = {}; int head = 0, count = 0; };

// ── FreeRTOS Mutex ────────────────────────────────────────────────────────────
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t bleMutex;
SemaphoreHandle_t dataMutex;

// ── Shared data ───────────────────────────────────────────────────────────────
volatile int32_t g_dispBPM  = 0;
volatile int32_t g_dispSpO2 = 0;
volatile bool    g_fingerOn = false;
volatile float   g_lastMag  = 1.0f;
volatile float   g_ax       = 0.0f;
volatile float   g_ay       = 0.0f;
volatile float   g_az       = 0.0f;

// Fall detection state (guarded by dataMutex)
volatile bool     g_fallDetected  = false;
volatile uint32_t g_fallAlertTime = 0;  // millis() khi phát hiện ngã
enum FallState { FALL_IDLE, FALL_FREEFALL, FALL_DETECTED };
volatile FallState g_fallState    = FALL_IDLE;
volatile uint32_t  g_freeFallTime = 0;

// ── RTC nội bộ (đồng bộ từ phone qua BLE) ────────────────────────────────────
// Sau khi nhận timestamp từ phone, ESP32 tự đếm thời gian bằng millis().
// Không cần module RTC phần cứng.
struct RTCState {
    // Epoch giây tại thời điểm đồng bộ (Unix-like, nhưng tính từ 2000-01-01)
    uint32_t syncEpoch   = 0;    // giây kể từ 2000-01-01 00:00:00
    uint32_t syncMillis  = 0;    // millis() lúc nhận timestamp từ phone
    bool     synced      = false;

    // Ngày tháng năm giờ phút giây tại thời điểm sync
    uint16_t year = 2000; uint8_t mon = 1; uint8_t day = 1;
    uint8_t  hour = 0;    uint8_t min = 0; uint8_t sec = 0;
};
volatile RTCState g_rtc;
SemaphoreHandle_t dtMutex;

// ── NimBLE objects ────────────────────────────────────────────────────────────
NimBLEServer*         pServer        = nullptr;
NimBLECharacteristic* pMpuChar       = nullptr;
NimBLECharacteristic* pHealthChar    = nullptr;
NimBLECharacteristic* pCommandChar   = nullptr;
NimBLECharacteristic* pDatetimeChar  = nullptr;
bool                  bleConnected   = false;

// ── MAX30102 ──────────────────────────────────────────────────────────────────
MAX30105  sensor;
uint32_t  irBuf[BUF_SIZE], redBuf[BUF_SIZE];
FIRState  firIR, firRed;
AGCState  agc;
MedianBuf mbpm, mspo2;
Biquad    bpHPF(0.9150f, -1.8300f, 0.9150f, -1.8226f, 0.8373f);
Biquad    bpLPF(0.1441f,  0.2882f, 0.1441f, -0.6776f, 0.2539f);

// ── OLED ──────────────────────────────────────────────────────────────────────
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, OLED_RST);

// ─────────────────────────────────────────────────────────────────────────────
//  BLE Callbacks
// ─────────────────────────────────────────────────────────────────────────────
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pSrv, NimBLEConnInfo& connInfo) override {
        bleConnected = true;
        Serial.println("[BLE] Connected");
    }
    void onDisconnect(NimBLEServer* pSrv, NimBLEConnInfo& connInfo, int reason) override {
        bleConnected = false;
        Serial.printf("[BLE] Disconnected, reason=%d → re-advertising\n", reason);
        NimBLEDevice::startAdvertising();
    }
};

class CommandCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
        std::string val = c->getValue();
        if (val == "RESET") Serial.println("[BLE] RESET command received");
    }
};

// ── Helpers RTC ───────────────────────────────────────────────────────────────
// Kiểm tra năm nhuận
static bool isLeap(uint16_t y) { return (y%4==0 && y%100!=0) || (y%400==0); }
static const uint8_t DAYS_IN_MONTH[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

// Parse chuỗi "YYYY-MM-DD HH:MM:SS" → cập nhật g_rtc
// Gọi từ DatetimeCallbacks (trong BLE callback, không hold i2cMutex)
static void rtcSync(const char* s) {
    // Cần ít nhất 19 ký tự: "YYYY-MM-DD HH:MM:SS"
    if (!s || strlen(s) < 19) return;
    uint16_t yr  = (s[0]-'0')*1000 + (s[1]-'0')*100 + (s[2]-'0')*10 + (s[3]-'0');
    uint8_t  mo  = (s[5]-'0')*10 + (s[6]-'0');
    uint8_t  dy  = (s[8]-'0')*10 + (s[9]-'0');
    uint8_t  hh  = (s[11]-'0')*10 + (s[12]-'0');
    uint8_t  mm  = (s[14]-'0')*10 + (s[15]-'0');
    uint8_t  ss  = (s[17]-'0')*10 + (s[18]-'0');

    // Validate
    if (mo < 1 || mo > 12 || dy < 1 || dy > 31) return;
    if (hh > 23 || mm > 59 || ss > 59)          return;

    xSemaphoreTake(dtMutex, portMAX_DELAY);
    g_rtc.year      = yr;
    g_rtc.mon       = mo;
    g_rtc.day       = dy;
    g_rtc.hour      = hh;
    g_rtc.min       = mm;
    g_rtc.sec       = ss;
    g_rtc.syncMillis = millis();
    g_rtc.synced    = true;
    xSemaphoreGive(dtMutex);

    Serial.printf("[RTC] Synced: %04d-%02d-%02d %02d:%02d:%02d\n",
                  yr, mo, dy, hh, mm, ss);
}

// Tính thời gian hiện tại dựa trên millis() delta kể từ lần sync
// Trả về "DD/MM HH:MM" vào buf[12]
static void rtcGetDisplay(char* buf, size_t len) {
    xSemaphoreTake(dtMutex, portMAX_DELAY);
    bool     synced = g_rtc.synced;
    uint16_t yr  = g_rtc.year;
    uint8_t  mo  = g_rtc.mon;
    uint8_t  dy  = g_rtc.day;
    uint8_t  hh  = g_rtc.hour;
    uint8_t  mm  = g_rtc.min;
    uint8_t  ss  = g_rtc.sec;
    uint32_t sm  = g_rtc.syncMillis;
    xSemaphoreGive(dtMutex);

    if (!synced) {
        snprintf(buf, len, "--/-- --:--");
        return;
    }

    // Thêm số giây đã trôi qua kể từ lần sync
    uint32_t elapsed = (millis() - sm) / 1000;
    ss += elapsed;

    // Tính tràn từ giây lên phút, giờ, ngày
    if (ss >= 60) { mm += ss / 60; ss %= 60; }
    if (mm >= 60) { hh += mm / 60; mm %= 60; }
    if (hh >= 24) {
        uint32_t extraDays = hh / 24;
        hh %= 24;
        // Cộng ngày tháng
        while (extraDays > 0) {
            uint8_t dMax = DAYS_IN_MONTH[mo - 1];
            if (mo == 2 && isLeap(yr)) dMax = 29;
            uint8_t remain = dMax - dy;
            if (extraDays <= remain) { dy += extraDays; extraDays = 0; }
            else { extraDays -= remain + 1; dy = 1; mo++; if (mo > 12) { mo = 1; yr++; } }
        }
    }

    snprintf(buf, len, "%02d/%02d %02d:%02d", dy, mo, hh, mm);
}

// Callback nhận datetime từ phone
// Phone gửi chuỗi "YYYY-MM-DD HH:MM:SS"  (19 ký tự)
class DatetimeCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
        std::string val = c->getValue();
        rtcSync(val.c_str());
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  BLE init
// ─────────────────────────────────────────────────────────────────────────────
void bleInit() {
    NimBLEDevice::init("ESP32-SmartWatch_TE");
    NimBLEDevice::setMTU(64);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* pService = pServer->createService(SERVICE_UUID);

    pMpuChar = pService->createCharacteristic(
        CHAR_MPU_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    pHealthChar = pService->createCharacteristic(
        CHAR_HEALTH_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    pCommandChar = pService->createCharacteristic(
        CHAR_COMMAND_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    pCommandChar->setCallbacks(new CommandCallbacks());

    // ── Characteristic datetime (write từ phone) ──────────────────────────────
    pDatetimeChar = pService->createCharacteristic(
        CHAR_DATETIME_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    pDatetimeChar->setCallbacks(new DatetimeCallbacks());

    pService->start();
    pServer->start();

    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->setName("ESP32-SmartWatch_TE");
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->enableScanResponse(true);
    NimBLEDevice::startAdvertising();
    Serial.println("[BLE] Advertising started (NimBLE 2.x)");
}

// ─────────────────────────────────────────────────────────────────────────────
//  BLE send helpers
// ─────────────────────────────────────────────────────────────────────────────
static void bleSendMPU(float* mags, int count) {
    if (!bleConnected || !pMpuChar) return;
    char buf[64];
    int pos = snprintf(buf, sizeof(buf), "M:");
    for (int i = 0; i < count && pos < (int)sizeof(buf) - 7; i++) {
        if (i > 0) buf[pos++] = '|';
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%.2f", mags[i]);
    }
    if (xSemaphoreTake(bleMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        pMpuChar->setValue((uint8_t*)buf, (size_t)pos);
        pMpuChar->notify();
        xSemaphoreGive(bleMutex);
    }
}

// Gửi health: "B:75,S:98,F:1,FALL:0"
static void bleSendHealth(int32_t bpm, int32_t spo2, bool finger, bool fall) {
    if (!bleConnected || !pHealthChar) return;
    char buf[40];
    int len = snprintf(buf, sizeof(buf), "B:%ld,S:%ld,F:%d,FALL:%d",
                       bpm, spo2, finger ? 1 : 0, fall ? 1 : 0);
    if (xSemaphoreTake(bleMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        pHealthChar->setValue((uint8_t*)buf, (size_t)len);
        pHealthChar->notify();
        xSemaphoreGive(bleMutex);
        Serial.printf("[BLE-Health] %s\n", buf);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  MPU6050 helpers
// ─────────────────────────────────────────────────────────────────────────────
static void mpuWriteReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg); Wire.write(val);
    Wire.endTransmission(true);
}

static bool mpuInit() {
    Wire.beginTransmission(MPU_ADDR);
    if (Wire.endTransmission(true) != 0) {
        Serial.printf("[MPU] Không tìm thấy tại 0x%02X\n", MPU_ADDR);
        return false;
    }
    mpuWriteReg(MPU_PWR_MGMT, 0x00);
    delay(100);
    Serial.printf("[MPU] OK tại 0x%02X\n", MPU_ADDR);
    return true;
}

struct MpuData { float ax, ay, az, mag; };

static MpuData mpuRead() {
    MpuData d = { g_ax, g_ay, g_az, g_lastMag };

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_ACCEL_XOUT);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, (uint8_t)true);
    if (Wire.available() < 6) return d;

    int16_t rx = (Wire.read() << 8) | Wire.read();
    int16_t ry = (Wire.read() << 8) | Wire.read();
    int16_t rz = (Wire.read() << 8) | Wire.read();
    d.ax  = rx / 16384.0f;
    d.ay  = ry / 16384.0f;
    d.az  = rz / 16384.0f;
    d.mag = sqrtf(d.ax*d.ax + d.ay*d.ay + d.az*d.az);
    return d;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Fall Detection logic (gọi mỗi lần có mẫu MPU mới)
//  Thuật toán: freefall (mag < 0.4g) → trong 500ms nếu impact (mag > 2.5g) → FALL
// ─────────────────────────────────────────────────────────────────────────────
static void updateFallDetect(float mag) {
    uint32_t now = millis();

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    FallState state = g_fallState;

    switch (state) {
        case FALL_IDLE:
            if (mag < FREE_FALL_THR) {
                g_fallState    = FALL_FREEFALL;
                g_freeFallTime = now;
            }
            break;

        case FALL_FREEFALL:
            if (mag > IMPACT_THR) {
                // Impact sau freefall → ngã!
                g_fallDetected  = true;
                g_fallAlertTime = now;
                g_fallState     = FALL_IDLE;
                Serial.printf("[FALL] DETECTED! mag=%.2f\n", mag);
            } else if (now - g_freeFallTime > FALL_WINDOW_MS) {
                // Hết cửa sổ thời gian, không phải ngã
                g_fallState = FALL_IDLE;
            }
            break;

        default:
            g_fallState = FALL_IDLE;
            break;
    }

    // Tắt cảnh báo sau FALL_ALERT_MS
    if (g_fallDetected && (now - g_fallAlertTime > FALL_ALERT_MS)) {
        g_fallDetected = false;
    }

    xSemaphoreGive(dataMutex);
}

// ─────────────────────────────────────────────────────────────────────────────
//  MAX30102 helpers
// ─────────────────────────────────────────────────────────────────────────────
static float applyBiquad(Biquad& f, float x) {
    float y = f.b0*x + f.z1;
    f.z1 = f.b1*x - f.a1*y + f.z2;
    f.z2 = f.b2*x - f.a2*y;
    return y;
}
static float applyFIR(FIRState& f, float x) {
    f.buf[f.head] = x;
    f.head = (f.head + 1) % FIR_LEN;
    float s = 0;
    for (int i = 0; i < FIR_LEN; i++) s += FIR_W[i] * f.buf[i];
    return s;
}
static int32_t bufMedian(int32_t* a, int n) {
    int32_t tmp[MED_LEN];
    memcpy(tmp, a, n * sizeof(int32_t));
    for (int i = 1; i < n; i++) {
        int32_t v = tmp[i]; int j = i - 1;
        while (j >= 0 && tmp[j] > v) { tmp[j+1] = tmp[j]; j--; }
        tmp[j+1] = v;
    }
    return tmp[n/2];
}
static void medPush(MedianBuf& m, int32_t v) {
    m.arr[m.head] = v;
    m.head = (m.head + 1) % MED_LEN;
    if (m.count < MED_LEN) m.count++;
}
static void agcTick(AGCState& agc, uint32_t rawIR, bool fingerOn) {
    if (!fingerOn) { agc.accum = 0; agc.n = 0; return; }
    agc.accum += rawIR;
    if (++agc.n < AGC_UPDATE_N) return;
    uint32_t dc = agc.accum / agc.n; agc.accum = 0; agc.n = 0;
    uint8_t nb = agc.br;
    if      (dc < AGC_DC_LOW  && agc.br < AGC_BR_MAX) nb = min((int)AGC_BR_MAX, agc.br + AGC_STEP);
    else if (dc > AGC_DC_HIGH && agc.br > AGC_BR_MIN) nb = max((int)AGC_BR_MIN, agc.br - AGC_STEP);
    else return;
    agc.br = nb;
    sensor.setPulseAmplitudeRed(agc.br);
    sensor.setPulseAmplitudeIR(agc.br);
}
static void collectSamples(int from, int count, bool& fingerOn) {
    for (int i = from; i < from + count; i++) {
        while (!sensor.available()) sensor.check();
        uint32_t rawIR  = sensor.getIR();
        uint32_t rawRed = sensor.getRed();
        sensor.nextSample();
        fingerOn = (rawIR > FINGER_THR);
        agcTick(agc, rawIR, fingerOn);
        irBuf[i]  = (uint32_t)max(0.0f, applyFIR(firIR,  (float)rawIR));
        redBuf[i] = (uint32_t)max(0.0f, applyFIR(firRed, (float)rawRed));
        applyBiquad(bpLPF, applyBiquad(bpHPF, (float)rawIR));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  TASK 1 — MPU6050 @ 66.7 Hz (Core 0, priority 3)
// ─────────────────────────────────────────────────────────────────────────────
void taskMPU(void* param) {
    TickType_t    xLastWake = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(MPU_INTERVAL_MS);

    float   magBatch[MPU_BATCH_SIZE];
    int     batchIdx = 0;
    MpuData lastMpu  = { 0.0f, 0.0f, 1.0f, 1.0f };

    while (true) {
        MpuData mpu;
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            mpu = mpuRead();
            xSemaphoreGive(i2cMutex);
            lastMpu = mpu;
        } else {
            mpu = lastMpu;
        }

        xSemaphoreTake(dataMutex, portMAX_DELAY);
        g_lastMag = mpu.mag;
        g_ax = mpu.ax; g_ay = mpu.ay; g_az = mpu.az;
        xSemaphoreGive(dataMutex);

        // Fall detection — gọi ngoài mutex vì updateFallDetect tự lấy mutex
        updateFallDetect(mpu.mag);

        magBatch[batchIdx++] = mpu.mag;
        if (batchIdx >= MPU_BATCH_SIZE) {
            bleSendMPU(magBatch, MPU_BATCH_SIZE);
            batchIdx = 0;
        }

        vTaskDelayUntil(&xLastWake, xPeriod);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  TASK 2 — MAX30102 HR/SpO2 (Core 1, priority 2)
// ─────────────────────────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────────
//  TASK 2 — MAX30102 HR/SpO2 (Core 1, priority 2)
// ─────────────────────────────────────────────────────────────────────────────
void taskMAX30102(void* param) {
    bool fingerOn   = false;
    bool prevFinger = false;

    // Init buffer: lấy từng SHIFT_SIZE mẫu, nhả mutex giữa các lần
    // để OLED và MPU không bị starvation trong lúc khởi tạo
    for (int offset = 0; offset < BUF_SIZE; offset += SHIFT_SIZE) {
        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
            collectSamples(offset, SHIFT_SIZE, fingerOn);
            xSemaphoreGive(i2cMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(5));  // nhường bus cho OLED/MPU giữa các batch
    }

    int32_t bpm = 0, spo2 = 0; int8_t bv = 0, sv = 0;
    maxim_heart_rate_and_oxygen_saturation(irBuf, BUF_SIZE, redBuf,
                                           &spo2, &sv, &bpm, &bv);

    uint32_t lastHealthSend = millis();

    while (true) {
        if (!fingerOn && prevFinger) {
            mbpm = MedianBuf{}; mspo2 = MedianBuf{};
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            g_dispBPM = g_dispSpO2 = 0;
            g_fingerOn = false;
            xSemaphoreGive(dataMutex);
            agc.br = 60; agc.accum = 0; agc.n = 0;
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                sensor.setPulseAmplitudeRed(60);
                sensor.setPulseAmplitudeIR(60);
                xSemaphoreGive(i2cMutex);
            }
        }
        prevFinger = fingerOn;

        for (int i = SHIFT_SIZE; i < BUF_SIZE; i++) {
            irBuf[i - SHIFT_SIZE]  = irBuf[i];
            redBuf[i - SHIFT_SIZE] = redBuf[i];
        }
        // collectSamples blocking ~250ms — mutex được giữ toàn bộ thời gian này
        // OLED task chờ tối đa 300ms nên vẫn lấy được mutex sau khi MAX nhả
        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
            collectSamples(BUF_SIZE - SHIFT_SIZE, SHIFT_SIZE, fingerOn);
            xSemaphoreGive(i2cMutex);
        }

        maxim_heart_rate_and_oxygen_saturation(irBuf, BUF_SIZE, redBuf,
                                               &spo2, &sv, &bpm, &bv);
        if (bv && bpm  >= BPM_MIN  && bpm  <= BPM_MAX)  medPush(mbpm,  bpm);
        if (sv && spo2 >= SPO2_MIN && spo2 <= SPO2_MAX) medPush(mspo2, spo2);

        int32_t dispBPM  = (fingerOn && mbpm.count  >= 3) ? bufMedian(mbpm.arr,  mbpm.count)  : 0;
        int32_t dispSpO2 = (fingerOn && mspo2.count >= 3) ? bufMedian(mspo2.arr, mspo2.count) : 0;

        xSemaphoreTake(dataMutex, portMAX_DELAY);
        g_dispBPM  = dispBPM;
        g_dispSpO2 = dispSpO2;
        g_fingerOn = fingerOn;
        xSemaphoreGive(dataMutex);

        Serial.printf("[MAX] bpm=%ld(%c) spo2=%ld(%c) disp=%ld/%ld finger=%c\n",
            bpm, bv?'V':'X', spo2, sv?'V':'X',
            dispBPM, dispSpO2, fingerOn?'Y':'N');

        uint32_t now = millis();
        if (now - lastHealthSend >= HEALTH_BLE_INTERVAL_MS) {
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            bool fall = g_fallDetected;
            xSemaphoreGive(dataMutex);
            bleSendHealth(dispBPM, dispSpO2, fingerOn, fall);
            lastHealthSend = now;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  OLED render — ~5 Hz (loop, Core 1)
//
//  Layout OLED 128×64 (font size1 = 6×8px, size2 = 12×16px):
//
//  ┌──────────────────────────────────┐
//  │ 14/03 09:45          [BLE●/○]   │  y=0  (size1, datetime + ble icon)
//  ├──────────────────────────────────┤
//  │ ♥ 75 bpm    SpO2 98%            │  y=16 (size2 cho số, icon size1)
//  │                                  │  y=32 (dòng kẻ ngang)
//  ├──────────────────────────────────┤
//  │ Mag: 1.02g   FALL: OK/!!!       │  y=40 (size1)
//  ├──────────────────────────────────┤  y=54
//  │ Place finger on sensor...        │  (nếu chưa đặt ngón)
//  └──────────────────────────────────┘
// ─────────────────────────────────────────────────────────────────────────────
static void renderOLED() {
    // Lấy snapshot dữ liệu
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    int32_t   bpm    = g_dispBPM;
    int32_t   spo2   = g_dispSpO2;
    bool      finger = g_fingerOn;
    float     mag    = g_lastMag;
    bool      fall   = g_fallDetected;
    xSemaphoreGive(dataMutex);

    char dtBuf[20];
    rtcGetDisplay(dtBuf, sizeof(dtBuf));

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(300)) != pdTRUE) return;

    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);

    // ── Dòng 1: Datetime + BLE icon (y=0, size1) ────────────────────────────
    oled.setTextSize(1);
    oled.setCursor(0, 1);
    oled.print(dtBuf);            // "DD/MM HH:MM"  ~66px
    // BLE icon: chấm tròn đặc = connected, vòng = advertising
    if (bleConnected) {
        oled.fillCircle(123, 3, 3, SSD1306_WHITE);  // chấm đặc
    } else {
        oled.drawCircle(123, 3, 3, SSD1306_WHITE);  // vòng rỗng
    }

    // ── Dòng kẻ ngang phân cách ──────────────────────────────────────────────
    oled.drawLine(0, 11, OLED_W - 1, 11, SSD1306_WHITE);

    if (!finger) {
        // ── Không đặt ngón: thông báo ────────────────────────────────────────
        oled.setTextSize(1);
        oled.setCursor(10, 18); oled.print("Place finger on");
        oled.setCursor(22, 28); oled.print("sensor...");

        // Mag + Fall vẫn hiện
        oled.setCursor(0, 42);
        oled.printf("Mag:%.2fg", mag);

        oled.setCursor(72, 42);
        if (fall) {
            oled.print("FALL:!!!");
        } else {
            oled.print("FALL: OK");
        }

        oled.setCursor(0, 54);
        oled.print(bleConnected ? "BLE: Connected" : "BLE: Advertising");

    } else {
        // ── Đặt ngón: hiển thị đầy đủ ────────────────────────────────────────

        // BPM — icon trái tim nhỏ + số lớn (size2)
        oled.setTextSize(1);
        oled.setCursor(20, 13);
        oled.print("\x03");   // ký tự trái tim trong font Adafruit GFX (char 3)
        // Nếu font không có trái tim, thay bằng "HR"
        // oled.print("HR");

        // BPM (trái - compact)
        oled.setTextSize(1);
        oled.setCursor(0, 12);
        oled.print("HR");   // hoặc icon

        oled.setTextSize(2);
        oled.setCursor(0, 20);
        if (bpm > 0) oled.print(bpm);
        else oled.print("--");

        oled.setTextSize(1);
        oled.setCursor(35, 28);
        oled.print("bpm");   // thay vì "bpm"

        // SpO2 — bên phải
        oled.setTextSize(1);
        oled.setCursor(80, 12);
        oled.print("O2");

        oled.setTextSize(2);
        oled.setCursor(80, 20);
        if (spo2 > 0) oled.print(spo2);
        else oled.print("--");

        oled.setTextSize(1);
        oled.setCursor(115, 28);
        oled.print("%");

        // ── Dòng kẻ giữa ──────────────────────────────────────────────────────
        oled.drawLine(0, 36, OLED_W - 1, 36, SSD1306_WHITE);

        // ── Dòng 3: Mag + Fall detect (y=43, size1) ───────────────────────────
        oled.setTextSize(1);
        oled.setCursor(0, 43);
        oled.printf("Mag:%.2fg", mag);

        oled.setCursor(72, 43);
        if (fall) {
            // Nhấp nháy khi ngã (blink 500ms)
            if ((millis() / 500) % 2 == 0) {
                oled.print("FALL:!!!");
            } else {
                oled.print("        ");  // trống để nhấp nháy
            }
        } else {
            oled.print("FALL: OK");
        }

        // ── Dòng 4: BLE status (y=54, size1) ─────────────────────────────────
        oled.setCursor(0, 54);
        oled.print(bleConnected ? "BLE: Connected  " : "BLE: Advertising");
    }

    oled.display();
    xSemaphoreGive(i2cMutex);
}

// ─────────────────────────────────────────────────────────────────────────────
//  TASK 3 — OLED render @ 5 Hz (Core 0, priority 1)
//
//  Chạy trên Core 0 (cùng taskMPU) thay vì Core 1 (nơi taskMAX chiếm bus).
//  Timeout i2cMutex = 300ms > collectSamples (~250ms) → luôn lấy được mutex
//  sau mỗi lần taskMAX nhả.
// ─────────────────────────────────────────────────────────────────────────────
void taskOLED(void* param) {
    // Chờ 500ms cho các task khác init xong trước khi bắt đầu render
    vTaskDelay(pdMS_TO_TICKS(500));
    while (true) {
        renderOLED();
        vTaskDelay(pdMS_TO_TICKS(200));  // ~5 Hz
    }
}
// ─────────────────────────────────────────────────────────────────────────────
static void i2cScan() {
    Serial.println("[I2C] Scanning bus...");
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  [I2C] Device at 0x%02X\n", addr);
            found++;
        }
    }
    if (found == 0) Serial.println("  [I2C] No devices found! Check wiring & pull-ups.");
    else            Serial.printf("  [I2C] %d device(s) found.\n", found);
}

// ─────────────────────────────────────────────────────────────────────────────
//  SETUP
//
//  ESP32-C3 notes:
//    • Pin 8/9 là GPIO thông thường, KHÔNG phải USB D-/D+.
//      USB CDC dùng internal USB PHY — không liên quan đến pin vật lý.
//    • Serial trên ESP32-C3 Arduino core mặc định map sang USB CDC (Serial0).
//      Cần delay ~1.5s sau Serial.begin() để host USB enumerate xong.
//    • Wire phải gọi begin() TRƯỚC bleInit() vì NimBLE stack chiếm thời gian
//      và có thể gây watchdog nếu I2C init bị trễ quá lâu.
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    // ── 1. Serial (USB CDC trên ESP32-C3) ────────────────────────────────────
    Serial.begin(115200);
    // Chờ USB CDC enumerate — bắt buộc trên ESP32-C3, không cần trên UART
#if ARDUINO_USB_CDC_ON_BOOT
    unsigned long t0 = millis();
    while (!Serial && millis() - t0 < 3000) { delay(10); }
#else
    delay(500);
#endif
    Serial.println("\n=== ESP32-C3 SmartWatch BOOT ===");

    // ── 2. I2C — explicit pin + 400 kHz Fast Mode ────────────────────────────
    // ESP32-C3: Wire.begin(sda, scl) dùng I2C0; tối đa 400 kHz ổn định
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000UL);
    delay(100);  // ổn định bus sau reset

    // Scan để debug phần cứng — tắt khi deploy nếu muốn
    i2cScan();

    // ── 3. Mutex (trước mọi task và BLE) ─────────────────────────────────────
    i2cMutex  = xSemaphoreCreateMutex();
    bleMutex  = xSemaphoreCreateMutex();
    dataMutex = xSemaphoreCreateMutex();
    dtMutex   = xSemaphoreCreateMutex();

    // ── 4. OLED — init sớm để hiện trạng thái boot ──────────────────────────
    Serial.println("[OLED] Initializing...");
    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("[OLED] FAILED — check address/wiring");
        // Không treo — tiếp tục boot không có OLED
    } else {
        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setTextColor(SSD1306_WHITE);
        oled.setCursor(10, 20); oled.print("Booting...");
        oled.display();
        Serial.println("[OLED] OK");
    }

    // ── 5. BLE ────────────────────────────────────────────────────────────────
    Serial.println("[BLE] Initializing...");
    bleInit();

    // ── 6. MAX30102 ───────────────────────────────────────────────────────────
    Serial.println("[MAX30102] Initializing...");
    if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("[MAX30102] FAILED — check wiring at 0x57");
        // Hiện lỗi trên OLED
        oled.clearDisplay();
        oled.setCursor(0, 0); oled.print("MAX30102 FAIL");
        oled.setCursor(0, 12); oled.print("Check 0x57 wiring");
        oled.display();
        while (true) { delay(1000); }
    }
    sensor.setup(60, 4, 2, 100, 411, 4096);
    Serial.println("[MAX30102] OK");

    // ── 7. MPU6050 ────────────────────────────────────────────────────────────
    Serial.println("[MPU6050] Initializing...");
    if (!mpuInit()) {
        Serial.println("[MPU6050] FAILED — check wiring at 0x68");
        // Tiếp tục boot — fall detect sẽ không hoạt động nhưng không crash
    }

    // ── 8. OLED boot done ─────────────────────────────────────────────────────
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(10, 20); oled.print("Initializing...");
    oled.display();

    // ── 9. Tasks ──────────────────────────────────────────────────────────────
    // Core 0: taskMPU (priority 3) + taskOLED (priority 1)
    // Core 1: taskMAX (priority 2)
    // Tách OLED sang Core 0 để tránh bị taskMAX preempt trên Core 1
    xTaskCreatePinnedToCore(taskMPU,      "taskMPU",  4096, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(taskOLED,     "taskOLED", 4096, nullptr, 1, nullptr, 0);
    xTaskCreatePinnedToCore(taskMAX30102, "taskMAX",  8192, nullptr, 2, nullptr, 1);

    Serial.println("[BOOT] All systems go!");
}

// ─────────────────────────────────────────────────────────────────────────────
//  LOOP — idle (Core 1), OLED đã được xử lý bởi taskOLED trên Core 0
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}