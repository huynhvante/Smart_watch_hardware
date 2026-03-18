#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

// ── BLE ───────────────────────────────────────────────────────────────────────
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID          "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_SENSOR_DATA_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_COMMAND_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a9"

BLEServer*         pServer        = nullptr;
BLECharacteristic* pSensorDataChar = nullptr;
BLECharacteristic* pCommandChar    = nullptr;
bool               bleConnected   = false;

// ── MPU6050 — dùng thư viện Adafruit_MPU6050 ─────────────────────────────────
struct Vec3 { float x, y, z; };

Adafruit_MPU6050 mpu;
bool  mpuOK          = false;
Vec3  accel          = {0, 0, 0};
float motionMag      = 0;
bool  motionDetected = false;

void mpuInit() {
    if (!mpu.begin()) {
        Serial.println("[WARN] MPU6050 not found");
        mpuOK = false;
        return;
    }
    // Cấu hình range và bandwidth
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);   // ±2g — độ nhạy cao nhất
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);         // ±250°/s
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);      // lọc nhiễu 21Hz

    mpuOK = true;
    Serial.println("[OK] MPU6050 ready (Adafruit lib)");
}

// Đọc gia tốc (m/s²) rồi chuyển sang g (chia 9.81)
Vec3 mpuRead() {
    sensors_event_t accelEvent, gyroEvent, tempEvent;
    mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
    // accelEvent.acceleration.x/y/z đơn vị m/s²
    // Chia 9.81 để ra g
    return {
        accelEvent.acceleration.x / 9.81f,
        accelEvent.acceleration.y / 9.81f,
        accelEvent.acceleration.z / 9.81f
    };
}

// ── BLE Callbacks ─────────────────────────────────────────────────────────────
class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* s) override {
        bleConnected = true;
        Serial.println("[BLE] Client connected");
    }
    void onDisconnect(BLEServer* s) override {
        bleConnected = false;
        Serial.println("[BLE] Client disconnected → restart advertising");
        BLEDevice::startAdvertising();
    }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
        std::string val = c->getValue();
        if (val.empty()) return;
        Serial.printf("[BLE] Command: %s\n", val.c_str());
        if (val == "RESET") Serial.println("[BLE] AGC reset");
    }
};

void bleInit() {
    BLEDevice::init("ESP32-HeartMonitor");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService* pService = pServer->createService(SERVICE_UUID);

    pSensorDataChar = pService->createCharacteristic(
        CHAR_SENSOR_DATA_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    pSensorDataChar->addDescriptor(new BLE2902());

    pCommandChar = pService->createCharacteristic(
        CHAR_COMMAND_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    pCommandChar->setCallbacks(new CommandCallbacks());

    pService->start();

    BLEAdvertising* pAdv = BLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->setScanResponse(true);
    BLEDevice::startAdvertising();
    // Tăng MTU lên 64 bytes để gửi chuỗi dài hơn 20 bytes
    BLEDevice::setMTU(64);
    Serial.println("[BLE] Advertising started (MTU=64)");
}

// Gửi dữ liệu BLE — format: "B:75,S:98,F:1,X:0.01,Y:-0.02,Z:1.00,M:0"
// M=1 nếu phát hiện chuyển động mạnh
void bleSendData(int32_t bpm, int32_t spo2, bool finger) {
    if (!bleConnected) return;
    char buf[64];
    snprintf(buf, sizeof(buf),
        "B:%ld,S:%ld,F:%d,X:%.2f,Y:%.2f,Z:%.2f,M:%d",
        bpm, spo2, finger ? 1 : 0,
        accel.x, accel.y, accel.z,
        motionDetected ? 1 : 0);
    Serial.printf("[BLE] Sending (%d bytes): %s\n", strlen(buf), buf);
    pSensorDataChar->setValue((uint8_t*)buf, strlen(buf));
    pSensorDataChar->notify();
}

// ── Cấu hình phần còn lại ────────────────────────────────────────────────────
#define I2C_SDA    8
#define I2C_SCL    9
#define OLED_ADDR  0x3C
#define OLED_W     128
#define OLED_H     64
#define OLED_RST   -1

#define BUF_SIZE   100
#define SHIFT_SIZE 25

#define AGC_DC_LOW    20000UL
#define AGC_DC_HIGH   50000UL
#define AGC_STEP      5
#define AGC_UPDATE_N  25
#define AGC_BR_MIN    10
#define AGC_BR_MAX    255
#define FINGER_THR    10000UL

#define FIR_LEN 5
static const float FIR_W[FIR_LEN] = { 0.10f, 0.20f, 0.40f, 0.20f, 0.10f };

#define BPM_MIN   40
#define BPM_MAX  200
#define SPO2_MIN  85
#define SPO2_MAX 100
#define MED_LEN   5

struct Biquad {
    float b0, b1, b2, a1, a2, z1, z2;
    Biquad(float b0, float b1, float b2, float a1, float a2)
        : b0(b0), b1(b1), b2(b2), a1(a1), a2(a2), z1(0), z2(0) {}
};
struct FIRState  { float buf[FIR_LEN] = {}; int head = 0; };
struct AGCState  { uint8_t br = 60; uint32_t accum = 0; int n = 0; };
struct MedianBuf { int32_t arr[MED_LEN] = {}; int head = 0, count = 0; };

static float applyBiquad(Biquad &f, float x) {
    float y = f.b0*x + f.z1;
    f.z1 = f.b1*x - f.a1*y + f.z2;
    f.z2 = f.b2*x - f.a2*y;
    return y;
}
static float applyFIR(FIRState &f, float x) {
    f.buf[f.head] = x;
    f.head = (f.head + 1) % FIR_LEN;
    float s = 0;
    for (int i = 0; i < FIR_LEN; i++) s += FIR_W[i] * f.buf[i];
    return s;
}
static int32_t bufMedian(int32_t *a, int n) {
    int32_t tmp[MED_LEN];
    memcpy(tmp, a, n * sizeof(int32_t));
    for (int i = 1; i < n; i++) {
        int32_t v = tmp[i]; int j = i - 1;
        while (j >= 0 && tmp[j] > v) { tmp[j+1] = tmp[j]; j--; }
        tmp[j+1] = v;
    }
    return tmp[n/2];
}
static void medPush(MedianBuf &m, int32_t v) {
    m.arr[m.head] = v;
    m.head = (m.head + 1) % MED_LEN;
    if (m.count < MED_LEN) m.count++;
}

static bool fingerOn = false, prevFinger = false;

static void agcTick(AGCState &agc, uint32_t rawIR, MAX30105 &s) {
    if (!fingerOn) { agc.accum = 0; agc.n = 0; return; }
    agc.accum += rawIR;
    if (++agc.n < AGC_UPDATE_N) return;
    uint32_t dc = agc.accum / agc.n;
    agc.accum = 0; agc.n = 0;
    uint8_t newBr = agc.br;
    if      (dc < AGC_DC_LOW  && agc.br < AGC_BR_MAX) newBr = min((int)AGC_BR_MAX, agc.br + AGC_STEP);
    else if (dc > AGC_DC_HIGH && agc.br > AGC_BR_MIN) newBr = max((int)AGC_BR_MIN, agc.br - AGC_STEP);
    else return;
    agc.br = newBr;
    s.setPulseAmplitudeRed(agc.br);
    s.setPulseAmplitudeIR(agc.br);
}

MAX30105         sensor;
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, OLED_RST);
uint32_t irBuf[BUF_SIZE], redBuf[BUF_SIZE];
FIRState  firIR, firRed;
AGCState  agc;
MedianBuf mbpm, mspo2;
Biquad bpHPF = { 0.9150f, -1.8300f, 0.9150f, -1.8226f, 0.8373f };
Biquad bpLPF = { 0.1441f,  0.2882f, 0.1441f, -0.6776f, 0.2539f };
int32_t dispBPM = 0, dispSpO2 = 0;

// ── Render OLED — thêm trang gia tốc ─────────────────────────────────────────
// Luân phiên 2 trang: BPM/SpO2 và Gia tốc
static uint8_t oledPage = 0;
static unsigned long lastPageSwitch = 0;
#define PAGE_INTERVAL 3000

static void renderOLED() {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);

    if (!fingerOn) {
        oled.setTextSize(1);
        oled.setCursor(14, 8);  oled.print("Place finger on");
        oled.setCursor(22, 20); oled.print("sensor...");
        // Gia tốc luôn hiện khi không đeo
        if (mpuOK) {
            oled.setCursor(0, 36);
            oled.print("X:"); oled.print(accel.x, 2);
            oled.print(" Y:"); oled.print(accel.y, 2);
            oled.setCursor(0, 48);
            oled.print("Z:"); oled.print(accel.z, 2);
            oled.print(motionDetected ? " [MOV]" : " [OK]");
        }
        oled.setCursor(0, 56);
        oled.print(bleConnected ? "BLE: Connected" : "BLE: Advertising");
        oled.display();
        return;
    }

    // Luân phiên trang
    if (millis() - lastPageSwitch > PAGE_INTERVAL) {
        oledPage = (oledPage + 1) % (mpuOK ? 2 : 1);
        lastPageSwitch = millis();
    }

    if (oledPage == 0) {
        // Trang 1: BPM + SpO2
        oled.setTextSize(2);
        oled.setCursor(0,  4); oled.print("BPM:");
        oled.setCursor(60, 4); oled.print(dispBPM  > 0 ? String(dispBPM)  : "--");
        oled.drawLine(0, 30, OLED_W, 30, SSD1306_WHITE);
        oled.setCursor(0,  34); oled.print("SpO2:");
        oled.setCursor(72, 34);
        if (dispSpO2 > 0) { oled.print(dispSpO2); oled.print("%"); }
        else              { oled.print("--"); }
        oled.setTextSize(1);
        oled.setCursor(0, 56);
        oled.print(bleConnected ? "BLE:OK" : "BLE:--");
        oled.print("  [1/2]");
    } else {
        // Trang 2: Gia tốc MPU6050
        oled.setTextSize(1);
        oled.setCursor(0, 0);  oled.print("-- Gia toc (g) --");
        oled.drawLine(0, 10, OLED_W, 10, SSD1306_WHITE);
        oled.setCursor(0, 14); oled.print("X: "); oled.print(accel.x, 3);
        oled.setCursor(0, 26); oled.print("Y: "); oled.print(accel.y, 3);
        oled.setCursor(0, 38); oled.print("Z: "); oled.print(accel.z, 3);
        oled.setCursor(0, 50);
        oled.print("|g|="); oled.print(motionMag, 2);
        oled.print(motionDetected ? "  [MOTION!]" : "  [Yen tinh]");
        oled.setCursor(100, 56); oled.print("[2/2]");
    }

    oled.display();
}

static void collect(int from, int count) {
    for (int i = from; i < from + count; i++) {
        while (!sensor.available()) sensor.check();
        uint32_t rawIR  = sensor.getIR();
        uint32_t rawRed = sensor.getRed();
        sensor.nextSample();
        fingerOn = (rawIR > FINGER_THR);
        agcTick(agc, rawIR, sensor);
        irBuf[i]  = (uint32_t)max(0.0f, applyFIR(firIR,  (float)rawIR));
        redBuf[i] = (uint32_t)max(0.0f, applyFIR(firRed, (float)rawRed));
        float acIR = applyBiquad(bpLPF, applyBiquad(bpHPF, (float)rawIR));
        (void)acIR;
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);

    bleInit();
    mpuInit();  // ← Thêm khởi động MPU6050

    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("OLED init failed"); while (true);
    }
    oled.clearDisplay();
    oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(20, 16); oled.print("Initializing...");
    oled.setCursor(10, 30); oled.print("BLE Advertising...");
    oled.setCursor(10, 44); oled.print(mpuOK ? "MPU6050: OK" : "MPU6050: N/A");
    oled.display();
    delay(1000);

    if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30102 not found"); while (true);
    }
    sensor.setup(60, 4, 2, 100, 411, 4096);

    collect(0, BUF_SIZE);
    int32_t bpm, spo2; int8_t bpmV, spo2V;
    maxim_heart_rate_and_oxygen_saturation(irBuf, BUF_SIZE, redBuf, &spo2, &spo2V, &bpm, &bpmV);
}

// ── Loop ──────────────────────────────────────────────────────────────────────
static unsigned long lastBleSend = 0;
static unsigned long lastMpuRead = 0;

void loop() {
    // ── Đọc MPU6050 mỗi 100ms ────────────────────────────────────────────────
    if (mpuOK && millis() - lastMpuRead >= 100) {
        lastMpuRead = millis();
        accel = mpuRead();
        motionMag = sqrt(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z);
        // Phát hiện chuyển động: lệch khỏi 1g quá 0.2g
        motionDetected = (fabs(motionMag - 1.0f) > 0.2f);
        Serial.printf("[MPU] X=%.3f Y=%.3f Z=%.3f |g|=%.3f %s\n",
            accel.x, accel.y, accel.z, motionMag,
            motionDetected ? "[MOTION]" : "[OK]");
    }

    // ── Reset khi tháo tay ────────────────────────────────────────────────────
    if (!fingerOn && prevFinger) {
        mbpm = MedianBuf{}; mspo2 = MedianBuf{};
        dispBPM = dispSpO2 = 0;
        agc.br = 60; agc.accum = 0; agc.n = 0;
        sensor.setPulseAmplitudeRed(60);
        sensor.setPulseAmplitudeIR(60);
    }
    prevFinger = fingerOn;

    // ── Thu thập MAX30102 ─────────────────────────────────────────────────────
    for (int i = SHIFT_SIZE; i < BUF_SIZE; i++) {
        irBuf[i  - SHIFT_SIZE] = irBuf[i];
        redBuf[i - SHIFT_SIZE] = redBuf[i];
    }
    collect(BUF_SIZE - SHIFT_SIZE, SHIFT_SIZE);

    int32_t bpm, spo2; int8_t bpmV, spo2V;
    maxim_heart_rate_and_oxygen_saturation(irBuf, BUF_SIZE, redBuf, &spo2, &spo2V, &bpm, &bpmV);

    if (bpmV  && bpm  >= BPM_MIN  && bpm  <= BPM_MAX)  medPush(mbpm,  bpm);
    if (spo2V && spo2 >= SPO2_MIN && spo2 <= SPO2_MAX)  medPush(mspo2, spo2);

    dispBPM  = (fingerOn && mbpm.count  >= 3) ? bufMedian(mbpm.arr,  mbpm.count)  : 0;
    dispSpO2 = (fingerOn && mspo2.count >= 3) ? bufMedian(mspo2.arr, mspo2.count) : 0;

    // ── Gửi BLE mỗi 1s ───────────────────────────────────────────────────────
    if (millis() - lastBleSend >= 1000) {
        lastBleSend = millis();
        bleSendData(dispBPM, dispSpO2, fingerOn);
    }

    // ── Log terminal ──────────────────────────────────────────────────────────
    Serial.printf("bpm=%ld(%c) spo2=%ld(%c) disp=%ld/%ld finger=%c br=%u ble=%c\n",
        bpm,  bpmV  ? 'V' : 'X',
        spo2, spo2V ? 'V' : 'X',
        dispBPM, dispSpO2,
        fingerOn ? 'Y' : 'N',
        agc.br,
        bleConnected ? 'Y' : 'N');

    renderOLED();
}