#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <NimBLEDevice.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ── UUID ──────────────────────────────────────────────────────────────────────
#define SERVICE_UUID      "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_MPU_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // notify batch magnitude
#define CHAR_HEALTH_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26aa"  // notify BPM/SpO2 mỗi 1s
#define CHAR_COMMAND_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"  // write

// ── Pin & OLED ────────────────────────────────────────────────────────────────
#define I2C_SDA   21
#define I2C_SCL   22
#define OLED_ADDR 0x3C
#define OLED_W    128
#define OLED_H    64
#define OLED_RST  -1

// ── MPU6050 ───────────────────────────────────────────────────────────────────
#define MPU_ADDR        0x68
#define MPU_PWR_MGMT    0x6B
#define MPU_ACCEL_XOUT  0x3B
// 66.7 Hz → 512 × 15 ms = 7.68 s < 8 s ✓
#define MPU_INTERVAL_MS 15
#define MPU_BATCH_SIZE  8   // gửi 1 gói BLE mỗi 8 × 15 ms = 120 ms

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
#define HEALTH_BLE_INTERVAL_MS 1000UL  // gửi health BLE mỗi 1 s

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

// ── NimBLE objects ────────────────────────────────────────────────────────────
NimBLEServer*         pServer      = nullptr;
NimBLECharacteristic* pMpuChar     = nullptr;
NimBLECharacteristic* pHealthChar  = nullptr;
NimBLECharacteristic* pCommandChar = nullptr;
bool                  bleConnected = false;

// ── MAX30102 (chỉ dùng trong taskMAX) ────────────────────────────────────────
MAX30105  sensor;
uint32_t  irBuf[BUF_SIZE], redBuf[BUF_SIZE];
FIRState  firIR, firRed;
AGCState  agc;
MedianBuf mbpm, mspo2;
Biquad    bpHPF(0.9150f, -1.8300f, 0.9150f, -1.8226f, 0.8373f);
Biquad    bpLPF(0.1441f,  0.2882f, 0.1441f, -0.6776f, 0.2539f);

// ── OLED ──────────────────────────────────────────────────────────────────────
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, OLED_RST);

//  NimBLE 2.x Callbacks — signature bắt buộc đúng với NimBLE 2.x
class ServerCallbacks : public NimBLEServerCallbacks {
    // NimBLE 2.x: (NimBLEServer*, NimBLEConnInfo&)
    void onConnect(NimBLEServer* pSrv, NimBLEConnInfo& connInfo) override {
        bleConnected = true;
        Serial.println("[BLE] Connected");
    }
    // NimBLE 2.x: (NimBLEServer*, NimBLEConnInfo&, int reason) — BẮT BUỘC có int reason
    void onDisconnect(NimBLEServer* pSrv, NimBLEConnInfo& connInfo, int reason) override {
        bleConnected = false;
        Serial.printf("[BLE] Disconnected, reason=%d → re-advertising\n", reason);
        NimBLEDevice::startAdvertising();
    }
};

class CommandCallbacks : public NimBLECharacteristicCallbacks {
    // NimBLE 2.x: (NimBLECharacteristic*, NimBLEConnInfo&)
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
        std::string val = c->getValue();
        if (val == "RESET") Serial.println("[BLE] RESET command received");
    }
};

//  BLE init
void bleInit() {
    NimBLEDevice::init("ESP32-SmartWatch_HUY");
    // MTU 64: payload 61 bytes, batch 8 mẫu ~42 bytes ✓
    NimBLEDevice::setMTU(64);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* pService = pServer->createService(SERVICE_UUID);

    // Characteristic 1: MPU magnitude batch (notify ~8 Hz)
    pMpuChar = pService->createCharacteristic(
        CHAR_MPU_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );

    // Characteristic 2: Health BPM/SpO2 (notify ~1 Hz)
    pHealthChar = pService->createCharacteristic(
        CHAR_HEALTH_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );

    // Characteristic 3: Command (write)
    pCommandChar = pService->createCharacteristic(
        CHAR_COMMAND_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    pCommandChar->setCallbacks(new CommandCallbacks());

    // NimBLE 2.x: start server (tự động start tất cả service)
    pServer->start();

    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    // NimBLE 2.x: enableScanResponse thay vì setScanResponse
    pAdv->enableScanResponse(true);
    NimBLEDevice::startAdvertising();
    Serial.println("[BLE] Advertising started (NimBLE 2.x)");
}

//  BLE send helpers

// Gửi batch magnitude: "M:9.81|9.82|...|9.80"  (taskMPU, mỗi 120 ms)
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
        Serial.printf("[BLE-MPU] %s (%d B)\n", buf, pos);
    }
}

// Gửi health: "B:75,S:98,F:1"  (taskMAX, mỗi 1 s)
static void bleSendHealth(int32_t bpm, int32_t spo2, bool finger) {
    if (!bleConnected || !pHealthChar) return;
    char buf[32];
    int len = snprintf(buf, sizeof(buf), "B:%ld,S:%ld,F:%d",
                       bpm, spo2, finger ? 1 : 0);
    if (xSemaphoreTake(bleMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        pHealthChar->setValue((uint8_t*)buf, (size_t)len);
        pHealthChar->notify();
        xSemaphoreGive(bleMutex);
        Serial.printf("[BLE-Health] %s\n", buf);
    }
}

//  MPU6050 helpers
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
    // Fallback: đọc shared data cũ — KHÔNG lấy mutex ở đây (caller đã hold i2cMutex)
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

//  MAX30102 helpers
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
// Gọi trong khi đang hold i2cMutex
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

//  TASK 1 — MPU6050 @ 66.7 Hz (Core 0, priority 3)
//
//  512 mẫu × 15 ms = 7.68 s < 8 s ✓
//  Batch 8 mẫu → gửi BLE "M:..." mỗi 120 ms
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
            mpu = lastMpu;  // Bus bận → dùng mẫu cũ, không bỏ slot thời gian
        }

        xSemaphoreTake(dataMutex, portMAX_DELAY);
        g_lastMag = mpu.mag;
        g_ax = mpu.ax; g_ay = mpu.ay; g_az = mpu.az;
        xSemaphoreGive(dataMutex);

        magBatch[batchIdx++] = mpu.mag;
        if (batchIdx >= MPU_BATCH_SIZE) {
            bleSendMPU(magBatch, MPU_BATCH_SIZE);
            batchIdx = 0;
        }

        vTaskDelayUntil(&xLastWake, xPeriod);
    }
}

//  TASK 2 — MAX30102 HR/SpO2 (Core 1, priority 2)
//
//  collectSamples blocking ≈ 250 ms (25 mẫu @ 100 SPS) → ~4 vòng/s
//  BLE health gửi mỗi HEALTH_BLE_INTERVAL_MS = 1000 ms bằng millis() timer
void taskMAX30102(void* param) {
    bool fingerOn  = false;
    bool prevFinger = false;

    // Khởi tạo buffer lần đầu
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        collectSamples(0, BUF_SIZE, fingerOn);
        xSemaphoreGive(i2cMutex);
    }
    int32_t bpm = 0, spo2 = 0; int8_t bv = 0, sv = 0;
    maxim_heart_rate_and_oxygen_saturation(irBuf, BUF_SIZE, redBuf,
                                           &spo2, &sv, &bpm, &bv);

    uint32_t lastHealthSend = millis();

    while (true) {
        // Reset khi rút ngón tay
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

        // Shift buffer
        for (int i = SHIFT_SIZE; i < BUF_SIZE; i++) {
            irBuf[i - SHIFT_SIZE]  = irBuf[i];
            redBuf[i - SHIFT_SIZE] = redBuf[i];
        }

        // Lấy 25 mẫu mới (blocking ≈ 250 ms)
        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
            collectSamples(BUF_SIZE - SHIFT_SIZE, SHIFT_SIZE, fingerOn);
            xSemaphoreGive(i2cMutex);
        }

        // Tính BPM/SpO2
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

        // Gửi BLE health mỗi ~1 s
        uint32_t now = millis();
        if (now - lastHealthSend >= HEALTH_BLE_INTERVAL_MS) {
            bleSendHealth(dispBPM, dispSpO2, fingerOn);
            lastHealthSend = now;
        }
        // Không cần vTaskDelay — collectSamples đã blocking ≈ 250 ms
    }
}

//  OLED render — ~5 Hz (loop, Core 1)
static void renderOLED() {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    int32_t bpm    = g_dispBPM;
    int32_t spo2   = g_dispSpO2;
    bool    finger = g_fingerOn;
    float   mag    = g_lastMag;
    xSemaphoreGive(dataMutex);

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(15)) != pdTRUE) return;

    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    if (!finger) {
        oled.setTextSize(1);
        oled.setCursor(14,16); oled.print("Place finger on");
        oled.setCursor(22,28); oled.print("sensor...");
        oled.setCursor(0,40);  oled.printf("|g|=%.3f", mag);
        oled.setCursor(0,52);  oled.print(bleConnected ? "BLE: Connected" : "BLE: Advertising");
    } else {
        oled.setTextSize(2);
        oled.setCursor(0, 4);  oled.print("BPM:");
        oled.setCursor(60,4);  oled.print(bpm  > 0 ? String(bpm)  : "--");
        oled.drawLine(0, 30, OLED_W, 30, SSD1306_WHITE);
        oled.setCursor(0, 36); oled.print("SpO2:");
        oled.setCursor(72,36);
        if (spo2 > 0) { oled.print(spo2); oled.print("%"); }
        else            oled.print("--");
        oled.setTextSize(1);
        oled.setCursor(0,56);
        oled.printf(bleConnected ? "BLE:OK |g|=%.2f" : "BLE:-- |g|=%.2f", mag);
    }
    oled.display();
    xSemaphoreGive(i2cMutex);
}

//  SETUP
void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);

    i2cMutex  = xSemaphoreCreateMutex();
    bleMutex  = xSemaphoreCreateMutex();
    dataMutex = xSemaphoreCreateMutex();

    bleInit();

    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("OLED init failed"); while (true);
    }
    oled.clearDisplay();
    oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(10,20); oled.print("Initializing...");
    oled.display();

    if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30102 init failed"); while (true);
    }
    sensor.setup(60, 4, 2, 100, 411, 4096);

    mpuInit();

    // taskMPU  → Core 0, priority 3  (giữ đúng 66.7 Hz)
    // taskMAX  → Core 1, priority 2  (blocking I/O)
    // loop()   → Core 1, priority 1  (OLED 5 Hz)
    xTaskCreatePinnedToCore(taskMPU,      "taskMPU", 4096, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(taskMAX30102, "taskMAX", 8192, nullptr, 2, nullptr, 1);
}

//  LOOP — OLED 5 Hz (Core 1)
void loop() {
    renderOLED();
    vTaskDelay(pdMS_TO_TICKS(200));
}