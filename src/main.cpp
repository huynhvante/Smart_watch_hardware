#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <NimBLEDevice.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <Arduino.h>

// UUID
#define SERVICE_UUID       "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_MPU_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_HEALTH_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26aa"
#define CHAR_COMMAND_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define CHAR_DATETIME_UUID "beb5483e-36e1-4688-b7f5-ea07361b26ab"

// Pin & OLED
#define I2C_SDA   8
#define I2C_SCL   9
#define OLED_ADDR 0x3C
#define OLED_W    128
#define OLED_H    64
#define OLED_RST  -1

// Button Pins — PULL UP, active LOW (nhấn nối GND, thả = HIGH)
// Dùng internal pull-up của ESP32-C3, nhấn nút nối GPIO xuống GND
#define BTN_SWITCH_PIN   10   // Nút 1: chuyển layout
#define BTN_CONFIRM_PIN  3    // Nút 2: confirm / đo
#define BTN_DEBOUNCE_MS  250  // ms chống rung

// MPU6050
#define MPU_ADDR        0x68
#define MPU_PWR_MGMT    0x6B
#define MPU_ACCEL_XOUT  0x3B
#define MPU_INTERVAL_MS 25
#define MPU_BATCH_SIZE  8

// MAX30102
#define BUF_SIZE               50
#define SHIFT_SIZE             10
#define AGC_DC_LOW             20000UL
#define AGC_DC_HIGH            40000UL
#define AGC_STEP               5
#define AGC_UPDATE_N           25
#define AGC_BR_MIN             10
#define AGC_BR_MAX             255
#define FINGER_THR             10000UL
#define FIR_LEN                5
#define BPM_MIN                40
#define BPM_MAX                200
#define SPO2_MIN               85
#define SPO2_MAX               100
#define MED_LEN                5
#define HEALTH_BLE_INTERVAL_MS 1000UL

// Fall Detection
#define FREE_FALL_THR  0.4f
#define IMPACT_THR     2.5f
#define FALL_WINDOW_MS 500
#define FALL_ALERT_MS  5000

// BPM History — 30 mẫu × 30 giây = 15 phút
#define BPM_HIST_SIZE        30
#define BPM_HIST_INTERVAL_MS 30000UL

// Chế độ đo chính xác (Layout 1, Nút 2)
#define MEAS_DURATION_MS 20000UL

// Firmware version
#define FW_VERSION "1.1.0"

// Số layout
#define NUM_LAYOUTS 3

// ── Structs ───────────────────────────────────────────────────────────────
static const float FIR_W[FIR_LEN] = { 0.10f, 0.20f, 0.40f, 0.20f, 0.10f };

struct Biquad {
    float b0,b1,b2,a1,a2,z1,z2;
    Biquad(float b0,float b1,float b2,float a1,float a2)
        : b0(b0),b1(b1),b2(b2),a1(a1),a2(a2),z1(0),z2(0) {}
};
struct FIRState  { float buf[FIR_LEN] = {}; int head = 0; };
struct AGCState  { uint8_t br = 60; uint32_t accum = 0; int n = 0; };
struct MedianBuf { int32_t arr[MED_LEN] = {}; int head = 0, count = 0; };

enum FallState  { FALL_IDLE, FALL_FREEFALL, FALL_DETECTED };
enum MeasState  { MEAS_IDLE, MEAS_RUNNING, MEAS_DONE };

// ── FreeRTOS Mutex ────────────────────────────────────────────────────────
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t bleMutex;
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t dtMutex;
SemaphoreHandle_t histMutex;

// ── Shared data (guarded by dataMutex) ────────────────────────────────────
volatile int32_t   g_dispBPM      = 0;
volatile int32_t   g_dispSpO2     = 0;
volatile bool      g_fingerOn     = false;
volatile float     g_lastMag      = 1.0f;
volatile float     g_ax = 0, g_ay = 0, g_az = 0;

volatile bool      g_fallDetected  = false;
volatile uint32_t  g_fallAlertTime = 0;
volatile FallState g_fallState     = FALL_IDLE;
volatile uint32_t  g_freeFallTime  = 0;

// ── UI State ──────────────────────────────────────────────────────────────
volatile int g_currentLayout = 0;  // 0=tổng quan, 1=nhịp tim, 2=gia tốc

// ── Button ISR flags (volatile, không dùng mutex) ─────────────────────────
volatile bool     g_btn1Pressed = false;
volatile bool     g_btn2Pressed = false;
volatile uint32_t g_btn1LastISR = 0;
volatile uint32_t g_btn2LastISR = 0;

void IRAM_ATTR isr_btn1() {
    uint32_t now = millis();
    if (now - g_btn1LastISR > BTN_DEBOUNCE_MS) {
        g_btn1LastISR = now;
        g_btn1Pressed = true;
    }
}

void IRAM_ATTR isr_btn2() {
    uint32_t now = millis();
    if (now - g_btn2LastISR > BTN_DEBOUNCE_MS) {
        g_btn2LastISR = now;
        g_btn2Pressed = true;
    }
}

// ── BPM History (guarded by histMutex) ────────────────────────────────────
int32_t  g_bpmHist[BPM_HIST_SIZE]  = {};
int      g_bpmHistHead  = 0;
int      g_bpmHistCount = 0;
int32_t  g_bpmMinToday  = 999;
int32_t  g_bpmMaxToday  = 0;
uint32_t g_lastHistUpdate = 0;

// ── Measurement state (guarded by dataMutex) ──────────────────────────────
volatile MeasState g_measState  = MEAS_IDLE;
volatile uint32_t  g_measStart  = 0;
volatile int32_t   g_measAccum  = 0;
volatile int       g_measCount  = 0;
volatile int32_t   g_measResult = 0;

// ── RTC nội bộ ────────────────────────────────────────────────────────────
struct RTCState {
    uint32_t syncEpoch  = 0;
    uint32_t syncMillis = 0;
    bool     synced     = false;
    uint16_t year = 2000; uint8_t mon = 1; uint8_t day = 1;
    uint8_t  hour = 0;    uint8_t min = 0; uint8_t sec = 0;
};
volatile RTCState g_rtc;

// ── NimBLE objects ────────────────────────────────────────────────────────
NimBLEServer*         pServer       = nullptr;
NimBLECharacteristic* pMpuChar      = nullptr;
NimBLECharacteristic* pHealthChar   = nullptr;
NimBLECharacteristic* pCommandChar  = nullptr;
NimBLECharacteristic* pDatetimeChar = nullptr;
bool                  bleConnected  = false;

// ── MAX30102 objects ──────────────────────────────────────────────────────
MAX30105  sensor;
uint32_t  irBuf[BUF_SIZE], redBuf[BUF_SIZE];
FIRState  firIR, firRed;
AGCState  agc;
MedianBuf mbpm, mspo2;

// ── OLED ──────────────────────────────────────────────────────────────────
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, OLED_RST);

// BLE Callbacks
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pSrv, NimBLEConnInfo& connInfo) override {
        bleConnected = true;
        Serial.println("[BLE] Connected");
    }
    void onDisconnect(NimBLEServer* pSrv, NimBLEConnInfo& connInfo, int reason) override {
        bleConnected = false;
        Serial.printf("[BLE] Disconnected reason=%d → re-advertising\n", reason);
        NimBLEDevice::startAdvertising();
    }
};

class CommandCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
        std::string val = c->getValue();
        if (val == "FALL:YES") {
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            g_fallDetected = true; g_fallAlertTime = millis();
            xSemaphoreGive(dataMutex);
        } else if (val == "FALL:NO") {
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            g_fallDetected = false;
            xSemaphoreGive(dataMutex);
        } else if (val == "RESET") {
            Serial.println("[BLE] RESET");
        }
    }
};

// RTC helpers
static bool isLeap(uint16_t y) { return (y%4==0 && y%100!=0) || (y%400==0); }
static const uint8_t DAYS_IN_MONTH[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

static void rtcSync(const char* s) {
    if (!s || strlen(s) < 19) return;
    uint16_t yr = (s[0]-'0')*1000+(s[1]-'0')*100+(s[2]-'0')*10+(s[3]-'0');
    uint8_t  mo = (s[5]-'0')*10+(s[6]-'0');
    uint8_t  dy = (s[8]-'0')*10+(s[9]-'0');
    uint8_t  hh = (s[11]-'0')*10+(s[12]-'0');
    uint8_t  mm = (s[14]-'0')*10+(s[15]-'0');
    uint8_t  ss = (s[17]-'0')*10+(s[18]-'0');
    if (mo<1||mo>12||dy<1||dy>31) return;
    if (hh>23||mm>59||ss>59)      return;
    xSemaphoreTake(dtMutex, portMAX_DELAY);
    g_rtc.year=yr; g_rtc.mon=mo; g_rtc.day=dy;
    g_rtc.hour=hh; g_rtc.min=mm; g_rtc.sec=ss;
    g_rtc.syncMillis=millis(); g_rtc.synced=true;
    xSemaphoreGive(dtMutex);
    Serial.printf("[RTC] Synced: %04d-%02d-%02d %02d:%02d:%02d\n",yr,mo,dy,hh,mm,ss);
}

static void rtcGetParts(char* timeBuf, size_t tLen, char* dateBuf, size_t dLen) {
    xSemaphoreTake(dtMutex, portMAX_DELAY);
    bool synced = g_rtc.synced;
    uint16_t yr=g_rtc.year; uint8_t mo=g_rtc.mon, dy=g_rtc.day;
    uint8_t hh=g_rtc.hour, mm=g_rtc.min, ss=g_rtc.sec;
    uint32_t sm=g_rtc.syncMillis;
    xSemaphoreGive(dtMutex);

    if (!synced) {
        snprintf(timeBuf,tLen,"--:--");
        snprintf(dateBuf,dLen,"--/--");
        return;
    }
    uint32_t elapsed = (millis()-sm)/1000;
    ss += elapsed;
    if (ss>=60){mm+=ss/60;ss%=60;}
    if (mm>=60){hh+=mm/60;mm%=60;}
    if (hh>=24){
        uint32_t ex=hh/24; hh%=24;
        while(ex>0){
            uint8_t dMax=DAYS_IN_MONTH[mo-1];
            if(mo==2&&isLeap(yr))dMax=29;
            uint8_t rem=dMax-dy;
            if(ex<=rem){dy+=ex;ex=0;}
            else{ex-=rem+1;dy=1;mo++;if(mo>12){mo=1;yr++;}}
        }
    }
    snprintf(timeBuf, tLen, "%02d:%02d", hh, mm);
    snprintf(dateBuf, dLen, "%02d/%02d", dy, mo);
}

class DatetimeCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
        std::string val = c->getValue();
        rtcSync(val.c_str());
    }
};

// BLE init & helpers
void bleInit() {
    NimBLEDevice::init("ESP32-SmartWatch_TE");
    NimBLEDevice::setMTU(64);
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    NimBLEService* pSvc = pServer->createService(SERVICE_UUID);
    pMpuChar    = pSvc->createCharacteristic(CHAR_MPU_UUID,     NIMBLE_PROPERTY::READ|NIMBLE_PROPERTY::NOTIFY);
    pHealthChar = pSvc->createCharacteristic(CHAR_HEALTH_UUID,  NIMBLE_PROPERTY::READ|NIMBLE_PROPERTY::NOTIFY);
    pCommandChar= pSvc->createCharacteristic(CHAR_COMMAND_UUID, NIMBLE_PROPERTY::WRITE|NIMBLE_PROPERTY::WRITE_NR);
    pCommandChar->setCallbacks(new CommandCallbacks());
    pDatetimeChar=pSvc->createCharacteristic(CHAR_DATETIME_UUID,NIMBLE_PROPERTY::WRITE|NIMBLE_PROPERTY::WRITE_NR);
    pDatetimeChar->setCallbacks(new DatetimeCallbacks());
    pSvc->start(); pServer->start();
    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->setName("ESP32-SmartWatch_TE");
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->enableScanResponse(true);
    NimBLEDevice::startAdvertising();
    Serial.println("[BLE] Advertising started");
}

static void bleSendMPU(float* mags, int count) {
    if (!bleConnected || !pMpuChar) return;
    char buf[64]; int pos = snprintf(buf, sizeof(buf), "M:");
    for (int i=0; i<count && pos<(int)sizeof(buf)-7; i++) {
        if(i>0) buf[pos++]='|';
        pos += snprintf(buf+pos, sizeof(buf)-pos, "%.2f", mags[i]);
    }
    if (xSemaphoreTake(bleMutex, pdMS_TO_TICKS(5))==pdTRUE) {
        pMpuChar->setValue((uint8_t*)buf,(size_t)pos); pMpuChar->notify();
        xSemaphoreGive(bleMutex);
    }
}

static void bleSendHealth(int32_t bpm, int32_t spo2, bool finger, bool fall) {
    if (!bleConnected || !pHealthChar) return;
    char buf[40];
    int len = snprintf(buf, sizeof(buf), "B:%ld,S:%ld,F:%d,FALL:%d",
                       bpm, spo2, finger?1:0, fall?1:0);
    if (xSemaphoreTake(bleMutex, pdMS_TO_TICKS(5))==pdTRUE) {
        pHealthChar->setValue((uint8_t*)buf,(size_t)len); pHealthChar->notify();
        xSemaphoreGive(bleMutex);
        Serial.printf("[BLE-Health] %s\n", buf);
    }
}

// MPU6050 helpers
static void mpuWriteReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val);
    Wire.endTransmission(true);
}
static bool mpuInit() {
    Wire.beginTransmission(MPU_ADDR);
    if (Wire.endTransmission(true)!=0) { Serial.printf("[MPU] Not found at 0x%02X\n",MPU_ADDR); return false; }
    mpuWriteReg(MPU_PWR_MGMT, 0x00); delay(100);
    Serial.printf("[MPU] OK at 0x%02X\n", MPU_ADDR); return true;
}
struct MpuData { float ax,ay,az,mag; };
static MpuData mpuRead() {
    MpuData d = { g_ax, g_ay, g_az, g_lastMag };
    Wire.beginTransmission(MPU_ADDR); Wire.write(MPU_ACCEL_XOUT);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR,(uint8_t)6,(uint8_t)true);
    if (Wire.available()<6) return d;
    int16_t rx=(Wire.read()<<8)|Wire.read();
    int16_t ry=(Wire.read()<<8)|Wire.read();
    int16_t rz=(Wire.read()<<8)|Wire.read();
    d.ax=rx/16384.0f; d.ay=ry/16384.0f; d.az=rz/16384.0f;
    d.mag=sqrtf(d.ax*d.ax+d.ay*d.ay+d.az*d.az);
    return d;
}

// Fall Detection
static void updateFallDetect(float mag) {
    uint32_t now = millis();
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    switch (g_fallState) {
        case FALL_IDLE:
            if (mag < FREE_FALL_THR) { g_fallState=FALL_FREEFALL; g_freeFallTime=now; }
            break;
        case FALL_FREEFALL:
            if (mag > IMPACT_THR) {
                g_fallDetected=true; g_fallAlertTime=now; g_fallState=FALL_IDLE;
                Serial.printf("[FALL] DETECTED! mag=%.2f\n", mag);
            } else if (now-g_freeFallTime > FALL_WINDOW_MS) {
                g_fallState=FALL_IDLE;
            }
            break;
        default: g_fallState=FALL_IDLE; break;
    }
    if (g_fallDetected && (now-g_fallAlertTime > FALL_ALERT_MS))
        g_fallDetected=false;
    xSemaphoreGive(dataMutex);
}

// MAX30102 helpers
static float applyFIR(FIRState& f, float x) {
    f.buf[f.head]=x; f.head=(f.head+1)%FIR_LEN;
    float s=0; for(int i=0;i<FIR_LEN;i++) s+=FIR_W[i]*f.buf[i]; return s;
}
static int32_t bufMedian(int32_t* a, int n) {
    int32_t tmp[MED_LEN]; memcpy(tmp,a,n*sizeof(int32_t));
    for(int i=1;i<n;i++){int32_t v=tmp[i];int j=i-1;while(j>=0&&tmp[j]>v){tmp[j+1]=tmp[j];j--;}tmp[j+1]=v;}
    return tmp[n/2];
}
static void medPush(MedianBuf& m, int32_t v) {
    m.arr[m.head]=v; m.head=(m.head+1)%MED_LEN; if(m.count<MED_LEN)m.count++;
}
static void agcTick(AGCState& agc, uint32_t rawIR, bool fingerOn) {
    if (!fingerOn) { agc.accum=0; agc.n=0; return; }
    agc.accum+=rawIR;
    if (++agc.n<AGC_UPDATE_N) return;
    uint32_t dc=agc.accum/agc.n; agc.accum=0; agc.n=0;
    uint8_t nb=agc.br;
    if      (dc<AGC_DC_LOW  && agc.br<AGC_BR_MAX) nb=min((int)AGC_BR_MAX,agc.br+AGC_STEP);
    else if (dc>AGC_DC_HIGH && agc.br>AGC_BR_MIN) nb=max((int)AGC_BR_MIN,agc.br-AGC_STEP);
    else return;
    agc.br=nb; sensor.setPulseAmplitudeRed(agc.br); sensor.setPulseAmplitudeIR(agc.br);
}

// ═══════════════════════════════════════════════════════════════════════════
// [FIX] collectSamples — release i2cMutex sau mỗi lần chờ sensor.
//
// Vấn đề gốc: taskMAX giữ i2cMutex với portMAX_DELAY trong khi
// while(!sensor.available()) sensor.check() spin không giới hạn → taskMPU
// timeout 2ms → không bao giờ đọc được → giá trị đóng băng.
//
// Giải pháp: tách thành 2 bước nhỏ, release mutex giữa mỗi sample
// để taskMPU có cơ hội chen vào. Logic đọc sensor hoàn toàn giữ nguyên.
// ═══════════════════════════════════════════════════════════════════════════
static void collectSamples(int from, int count, bool& fingerOn) {
    for (int i = from; i < from + count; i++) {

        // B1: Chờ sensor có data — release mutex giữa mỗi lần poll
        // taskMPU (timeout 2ms) có thể chen vào trong khoảng vTaskDelay(1)
        bool ready = false;
        while (!ready) {
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                sensor.check();
                ready = sensor.available();
                xSemaphoreGive(i2cMutex);  // release ngay, không giữ lâu
            }
            if (!ready) vTaskDelay(1);      // yield 1 tick → taskMPU chạy được
        }

        // B2: Đọc 1 sample — mutex chỉ giữ vài µs
        uint32_t rawIR = 0, rawRed = 0;
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            rawIR  = sensor.getIR();
            rawRed = sensor.getRed();
            sensor.nextSample();
            xSemaphoreGive(i2cMutex);      // release ngay
        }

        // Phần xử lý data giữ nguyên hoàn toàn như code gốc
        fingerOn = (rawIR > FINGER_THR);
        agcTick(agc, rawIR, fingerOn);
        irBuf[i]  = (uint32_t)max(0.0f, applyFIR(firIR,  (float)rawIR));
        redBuf[i] = (uint32_t)max(0.0f, applyFIR(firRed, (float)rawRed));
    }
}

// BPM History helpers
static void bpmHistoryPush(int32_t bpm) {
    xSemaphoreTake(histMutex, portMAX_DELAY);
    g_bpmHist[g_bpmHistHead] = bpm;
    g_bpmHistHead = (g_bpmHistHead + 1) % BPM_HIST_SIZE;
    if (g_bpmHistCount < BPM_HIST_SIZE) g_bpmHistCount++;
    if (bpm < g_bpmMinToday || g_bpmMinToday == 999) g_bpmMinToday = bpm;
    if (bpm > g_bpmMaxToday) g_bpmMaxToday = bpm;
    xSemaphoreGive(histMutex);
}

// Button Handler — ISR flags, KHÔNG cần mutex, KHÔNG block
static void handleButtons() {
    if (g_btn1Pressed) {
        g_btn1Pressed   = false;
        g_currentLayout = (g_currentLayout + 1) % NUM_LAYOUTS;
        Serial.printf("[BTN] Layout → %d\n", (int)g_currentLayout);
    }

    if (g_btn2Pressed) {
        g_btn2Pressed = false;
        int layout = g_currentLayout;

        if (layout == 1) {
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            MeasState ms = g_measState;
            xSemaphoreGive(dataMutex);

            if (ms == MEAS_IDLE || ms == MEAS_DONE) {
                xSemaphoreTake(dataMutex, portMAX_DELAY);
                g_measState  = MEAS_RUNNING;
                g_measStart  = millis();
                g_measAccum  = 0;
                g_measCount  = 0;
                g_measResult = 0;
                xSemaphoreGive(dataMutex);
                Serial.println("[MEAS] Started 20s measurement");
            }

        } else if (layout == 2) {
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            if (g_fallDetected) {
                g_fallDetected = false;
                g_fallState    = FALL_IDLE;
                Serial.println("[BTN] Fall alert reset by user");
            }
            xSemaphoreGive(dataMutex);
        }
    }
}

// OLED Draw helpers
static void drawPageDots(int y) {
    for (int i = 0; i < NUM_LAYOUTS; i++) {
        int px = 103 + i * 8;
        if (i == (int)g_currentLayout) oled.fillCircle(px, y, 2, SSD1306_WHITE);
        else                           oled.drawCircle(px, y, 2, SSD1306_WHITE);
    }
}

static void drawProgressBar(int x, int y, int w, int h, int pct) {
    pct = constrain(pct, 0, 100);
    oled.drawRect(x, y, w, h, SSD1306_WHITE);
    int fillW = (w - 2) * pct / 100;
    if (fillW > 0) oled.fillRect(x+1, y+1, fillW, h-2, SSD1306_WHITE);
}

static void drawBpmChart(int cx, int cy, int cw, int ch) {
    xSemaphoreTake(histMutex, portMAX_DELAY);
    int     count = g_bpmHistCount;
    int     head  = g_bpmHistHead;
    int32_t hist[BPM_HIST_SIZE];
    memcpy(hist, g_bpmHist, sizeof(g_bpmHist));
    xSemaphoreGive(histMutex);

    oled.drawRect(cx, cy, cw, ch, SSD1306_WHITE);

    if (count == 0) {
        oled.setTextSize(1);
        oled.setCursor(cx + 20, cy + ch/2 - 4);
        oled.print("No data yet");
        return;
    }

    int innerW = cw - 2;
    int innerH = ch - 2;
    float bw = (float)innerW / BPM_HIST_SIZE;

    for (int i = 0; i < count; i++) {
        int idx   = (head - count + i + BPM_HIST_SIZE) % BPM_HIST_SIZE;
        int32_t v = hist[idx];
        if (v <= 0) continue;
        int barH = map(constrain(v, 50, 150), 50, 150, 1, innerH);
        int bx   = cx + 1 + (int)(i * bw);
        int by   = cy + ch - 1 - barH;
        int bwi  = max(1, (int)bw);
        oled.fillRect(bx, by, bwi, barH, SSD1306_WHITE);
    }
}

// RENDER LAYOUT 0 — Tổng quan
static void renderLayout0(int32_t bpm, int32_t spo2, bool finger, float mag, bool fall) {
    char timeBuf[8], dateBuf[8];
    rtcGetParts(timeBuf, sizeof(timeBuf), dateBuf, sizeof(dateBuf));

    oled.setTextSize(2);
    oled.setCursor(0, 0);
    oled.print(timeBuf);

    oled.setTextSize(1);
    oled.setCursor(68, 4);
    oled.print(dateBuf);

    drawPageDots(3);
    oled.drawLine(0, 17, OLED_W-1, 17, SSD1306_WHITE);

    if (!finger) {
        oled.setTextSize(1);
        oled.setCursor(18, 24); oled.print("wear device on");
        oled.setCursor(28, 34); oled.print("your wrist...");
    } else {
        oled.setTextSize(1); oled.setCursor(0, 19);  oled.print("HR");
        oled.setTextSize(2); oled.setCursor(0, 27);
        if (bpm > 0) oled.print(bpm); else oled.print("--");
        oled.setTextSize(1); oled.setCursor(36, 36); oled.print("bpm");

        oled.setTextSize(1); oled.setCursor(72, 19); oled.print("SpO2");
        oled.setTextSize(2); oled.setCursor(72, 27);
        if (spo2 > 0) oled.print(spo2); else oled.print("--");
        oled.setTextSize(1); oled.setCursor(100, 36); oled.print("%");
    }

    oled.drawLine(0, 44, OLED_W-1, 44, SSD1306_WHITE);

    oled.setTextSize(1);
    oled.setCursor(0, 47);
    oled.printf("M=%.2f==>", mag);

    oled.setCursor(54, 47);
    if (fall) {
        if ((millis()/400)%2 == 0) oled.print("!FALL!");
    } else {
        oled.print("SAFE");
    }

    oled.setCursor(0, 56);
    oled.print(bleConnected ? "BLE:Connected" : "BLE:Disconnected...");
}

// RENDER LAYOUT 1 — Nhịp tim chi tiết
static void renderLayout1(int32_t bpm, bool finger) {
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print("HEART RATE");
    drawPageDots(3);
    oled.drawLine(0, 9, OLED_W-1, 9, SSD1306_WHITE);

    drawBpmChart(0, 10, 128, 20);

    oled.drawLine(0, 31, OLED_W-1, 31, SSD1306_WHITE);

    xSemaphoreTake(histMutex, portMAX_DELAY);
    int32_t bMin = g_bpmMinToday;
    int32_t bMax = g_bpmMaxToday;
    xSemaphoreGive(histMutex);

    oled.setTextSize(1);
    oled.setCursor(0, 33);
    if (bMin == 999) oled.print("Min:--  Max:--");
    else             oled.printf("Min:%ld  Max:%ld", bMin, bMax);

    oled.drawLine(0, 42, OLED_W-1, 42, SSD1306_WHITE);

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    MeasState ms     = g_measState;
    uint32_t  mStart = g_measStart;
    int32_t   mRes   = g_measResult;
    xSemaphoreGive(dataMutex);

    oled.setTextSize(1);

    if (ms == MEAS_IDLE) {
        oled.setCursor(0, 44);
        oled.print("HR: ");
        if (finger && bpm > 0) oled.printf("%ld BPM", bpm);
        else                   oled.print("-- BPM");

        oled.setCursor(0, 55);
        oled.print("Press BTN2 to start");

    } else if (ms == MEAS_RUNNING) {
        uint32_t elapsed = millis() - mStart;
        if (elapsed > MEAS_DURATION_MS) elapsed = MEAS_DURATION_MS;
        int pct = (int)(elapsed * 100UL / MEAS_DURATION_MS);

        oled.setCursor(0, 44);
        if ((millis()/600)%2 == 0) oled.print("Keep still!");
        else                       oled.print("Measuring...");

        drawProgressBar(0, 54, 100, 8, pct);

        oled.setCursor(104, 55);
        oled.printf("%d%%", pct);

    } else { // MEAS_DONE
        oled.setCursor(0, 44);
        if (mRes > 0) oled.printf("Result: %ld BPM", mRes);
        else          oled.print("Result: Error!");

        oled.setCursor(0, 55);
        oled.print("[BTN2]measure again");
    }
}

// RENDER LAYOUT 2 — Gia tốc & Té ngã
static void renderLayout2(float mag, float ax, float ay, float az, bool fall) {
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print("FALL DETECTION");
    drawPageDots(3);
    oled.drawLine(0, 9, OLED_W-1, 9, SSD1306_WHITE);

    oled.setTextSize(1);
    oled.setCursor(0, 11);
    if (fall) {
        if ((millis()/400)%2 == 0) oled.print("!!Risk: HIGH!!");
    } else {
        oled.print("Risk: LOW");
    }

    oled.setCursor(0, 21);
    oled.printf("Mag:%.2fg", mag);
    oled.setCursor(70, 21);
    oled.printf("X:% .1f", ax);
    oled.setCursor(0, 30);
    oled.printf("Y:% .1f  Z:% .1f", ay, az);

    oled.drawLine(0, 40, OLED_W-1, 40, SSD1306_WHITE);

    uint32_t upSec = millis() / 1000;
    uint32_t upH   = upSec / 3600;
    uint32_t upM   = (upSec % 3600) / 60;
    oled.setCursor(0, 42);
    oled.printf("Uptime: %luh %02lum", upH, upM);

    oled.setCursor(0, 52);
    oled.print(bleConnected ? "BLE:Connected" : "BLE:Disconnected...");

    oled.setCursor(80, 62);
    oled.print("v" FW_VERSION);

    if (fall) {
        oled.setCursor(0, 56);
        if ((millis()/600)%2 == 0) oled.print("[BTN2]Reset alert");
    }
}

// renderOLED — dispatcher chính
static void renderOLED() {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    int32_t bpm   = g_dispBPM;
    int32_t spo2  = g_dispSpO2;
    bool    finger= g_fingerOn;
    float   mag   = g_lastMag;
    float   ax    = g_ax, ay = g_ay, az = g_az;
    bool    fall  = g_fallDetected;
    xSemaphoreGive(dataMutex);

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(300)) != pdTRUE) return;

    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);

    switch ((int)g_currentLayout) {
        case 0:  renderLayout0(bpm, spo2, finger, mag, fall); break;
        case 1:  renderLayout1(bpm, finger);                  break;
        case 2:  renderLayout2(mag, ax, ay, az, fall);        break;
        default: renderLayout0(bpm, spo2, finger, mag, fall); break;
    }

    oled.display();
    xSemaphoreGive(i2cMutex);
}

// TASK 1 — MPU6050 @ ~40 Hz (priority 1)
// Giữ nguyên hoàn toàn, timeout 2ms đủ vì collectSamples đã release mutex
void taskMPU(void* param) {
    TickType_t       xLastWake = xTaskGetTickCount();
    const TickType_t xPeriod   = pdMS_TO_TICKS(MPU_INTERVAL_MS);
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
        g_lastMag=mpu.mag; g_ax=mpu.ax; g_ay=mpu.ay; g_az=mpu.az;
        xSemaphoreGive(dataMutex);

        updateFallDetect(mpu.mag);

        magBatch[batchIdx++] = mpu.mag;
        if (batchIdx >= MPU_BATCH_SIZE) {
            bleSendMPU(magBatch, MPU_BATCH_SIZE);
            batchIdx = 0;
        }

        vTaskDelayUntil(&xLastWake, xPeriod);
    }
}

// TASK 2 — MAX30102 HR/SpO2 (priority 2)
// [FIX] Bỏ xSemaphoreTake/Give i2cMutex bên ngoài collectSamples
//       vì collectSamples đã tự quản lý mutex bên trong
void taskMAX30102(void* param) {
    bool fingerOn = false, prevFinger = false;

    // Fill buffer ban đầu — gọi trực tiếp, không bọc mutex ngoài
    for (int offset = 0; offset < BUF_SIZE; offset += SHIFT_SIZE) {
        collectSamples(offset, SHIFT_SIZE, fingerOn);  // [FIX] bỏ mutex ngoài
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    int32_t bpm=0, spo2=0; int8_t bv=0, sv=0;
    maxim_heart_rate_and_oxygen_saturation(irBuf,BUF_SIZE,redBuf,&spo2,&sv,&bpm,&bv);

    uint32_t lastHealthSend = millis();

    while (true) {
        if (!fingerOn && prevFinger) {
            mbpm=MedianBuf{}; mspo2=MedianBuf{};
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            g_dispBPM=g_dispSpO2=0; g_fingerOn=false;
            if (g_measState == MEAS_RUNNING) g_measState = MEAS_IDLE;
            xSemaphoreGive(dataMutex);
            agc.br=60; agc.accum=0; agc.n=0;
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20))==pdTRUE) {
                sensor.setPulseAmplitudeRed(60); sensor.setPulseAmplitudeIR(60);
                xSemaphoreGive(i2cMutex);
            }
        }
        prevFinger = fingerOn;

        for (int i=SHIFT_SIZE; i<BUF_SIZE; i++) {
            irBuf[i-SHIFT_SIZE]=irBuf[i]; redBuf[i-SHIFT_SIZE]=redBuf[i];
        }

        collectSamples(BUF_SIZE-SHIFT_SIZE, SHIFT_SIZE, fingerOn);  // [FIX] bỏ mutex ngoài

        maxim_heart_rate_and_oxygen_saturation(irBuf,BUF_SIZE,redBuf,&spo2,&sv,&bpm,&bv);
        if (bv && bpm>=BPM_MIN && bpm<=BPM_MAX)    medPush(mbpm,  bpm);
        if (sv && spo2>=SPO2_MIN && spo2<=SPO2_MAX) medPush(mspo2, spo2);

        int32_t dispBPM  = (fingerOn && mbpm.count >=3) ? bufMedian(mbpm.arr,  mbpm.count)  : 0;
        int32_t dispSpO2 = (fingerOn && mspo2.count>=3) ? bufMedian(mspo2.arr, mspo2.count) : 0;

        xSemaphoreTake(dataMutex, portMAX_DELAY);
        g_dispBPM=dispBPM; g_dispSpO2=dispSpO2; g_fingerOn=fingerOn;
        xSemaphoreGive(dataMutex);

        Serial.printf("[MAX] bpm=%ld(%c) spo2=%ld(%c) disp=%ld/%ld finger=%c\n",
            bpm,bv?'V':'X',spo2,sv?'V':'X',dispBPM,dispSpO2,fingerOn?'Y':'N');

        // BPM History (mỗi 30s)
        uint32_t nowMs = millis();
        if (nowMs - g_lastHistUpdate >= BPM_HIST_INTERVAL_MS && fingerOn && dispBPM > 0) {
            g_lastHistUpdate = nowMs;
            bpmHistoryPush(dispBPM);
        }

        // Chế độ đo chính xác 20s
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        MeasState ms = g_measState;
        xSemaphoreGive(dataMutex);

        if (ms == MEAS_RUNNING) {
            uint32_t elapsed = millis() - g_measStart;
            if (elapsed >= MEAS_DURATION_MS) {
                xSemaphoreTake(dataMutex, portMAX_DELAY);
                g_measResult = (g_measCount > 0) ? (g_measAccum / g_measCount) : 0;
                g_measState  = MEAS_DONE;
                xSemaphoreGive(dataMutex);
                Serial.printf("[MEAS] Done. Result=%ld BPM (%d samples)\n",
                              g_measResult, g_measCount);
            } else if (fingerOn && dispBPM > 0) {
                xSemaphoreTake(dataMutex, portMAX_DELAY);
                g_measAccum += dispBPM;
                g_measCount++;
                xSemaphoreGive(dataMutex);
            }
        }

        // Gửi health qua BLE mỗi 1s
        if (nowMs - lastHealthSend >= HEALTH_BLE_INTERVAL_MS) {
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            bool fall = g_fallDetected;
            xSemaphoreGive(dataMutex);
            bleSendHealth(dispBPM, dispSpO2, fingerOn, fall);
            lastHealthSend = nowMs;
        }

        vTaskDelay(1);
    }
}

// TASK 3 — OLED render @ 5 Hz (priority 1)
void taskOLED(void* param) {
    vTaskDelay(pdMS_TO_TICKS(100));

    while (true) {
        handleButtons();
        renderOLED();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// I2C scan (debug)
static void i2cScan() {
    Serial.println("[I2C] Scanning...");
    uint8_t found = 0;
    for (uint8_t addr=1; addr<127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission()==0) {
            Serial.printf("  [I2C] Device at 0x%02X\n", addr); found++;
        }
    }
    if (!found) Serial.println("  [I2C] No devices found!");
    else        Serial.printf("  [I2C] %d device(s).\n", found);
}

// SETUP
void setup() {
    // 1. Serial
    Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
    unsigned long t0 = millis();
    while (!Serial && millis()-t0 < 3000) delay(10);
#else
    delay(500);
#endif
    Serial.println("\n=== ESP32-C3 SmartWatch BOOT v" FW_VERSION " ===");

    // 2. I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000UL);
    Wire.setTimeOut(3);  // [FIX] tránh Wire treo khi LiPo/powerbank boot chậm
    delay(100);
    i2cScan();

    // 3. Buttons — PULL UP, active LOW
    pinMode(BTN_SWITCH_PIN,  INPUT_PULLUP);
    pinMode(BTN_CONFIRM_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(BTN_SWITCH_PIN),  isr_btn1, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN_CONFIRM_PIN), isr_btn2, FALLING);

    Serial.printf("[BTN] Switch=GPIO%d  Confirm=GPIO%d (pull-up, active LOW)\n",
                  BTN_SWITCH_PIN, BTN_CONFIRM_PIN);

    // 4. Mutex
    i2cMutex  = xSemaphoreCreateMutex();
    bleMutex  = xSemaphoreCreateMutex();
    dataMutex = xSemaphoreCreateMutex();
    dtMutex   = xSemaphoreCreateMutex();
    histMutex = xSemaphoreCreateMutex();

    // 5. OLED
    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("[OLED] FAILED");
    } else {
        oled.clearDisplay();
        oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
        oled.setCursor(28, 24); oled.print("Booting v" FW_VERSION);
        oled.display();
        Serial.println("[OLED] OK");
    }

    // 6. BLE
    bleInit();

    // 7. MAX30102
    if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("[MAX30102] FAILED");
        oled.clearDisplay();
        oled.setCursor(0,0); oled.print("MAX30102 FAIL");
        oled.setCursor(0,12); oled.print("Check 0x57 wiring");
        oled.display();
        while (true) delay(1000);
    }
    sensor.setup(60, 4, 2, 100, 411, 4096);
    Serial.println("[MAX30102] OK");

    // 8. MPU6050
    if (!mpuInit()) Serial.println("[MPU6050] FAILED — fall detect disabled");

    // 9. OLED boot done
    oled.clearDisplay();
    oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(22, 20); oled.print("Initializing...");
    oled.setCursor(5, 32);  oled.print("BTN1:Switch Layout");
    oled.setCursor(5, 44);  oled.print("BTN2:Measure/Confirm");
    oled.display();
    delay(100);

    // 10. FreeRTOS Tasks
    xTaskCreate(taskMPU,     "taskMPU",  4096, nullptr, 1, nullptr);
    xTaskCreate(taskOLED,    "taskOLED", 4096, nullptr, 1, nullptr);
    xTaskCreate(taskMAX30102,"taskMAX",  8192, nullptr, 1, nullptr);

    Serial.println("[BOOT] All systems go!");
    Serial.printf("[BOOT] BTN1(GPIO%d)=Switch layout (pull-up)\n", BTN_SWITCH_PIN);
    Serial.printf("[BOOT] BTN2(GPIO%d)=Confirm/Measure (pull-up)\n", BTN_CONFIRM_PIN);
}

// LOOP — idle
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
