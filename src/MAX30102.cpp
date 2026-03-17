#include "MAX30102.h"

// ── Static array definitions (phải khai báo ở .cpp, không được ở .h) ──────
const uint8_t MAX30102::INT_CFG_REG[] = {
  0x02,   // FIFO Almost Full Interrupt Enable
  0x03,   // Temperature Ready Interrupt Enable
  0x02,   // PPG Ready Interrupt Enable
  0x02,   // Ambient Light Cancellation Overflow Interrupt Enable
  0xFF    // Power Ready (không tồn tại)
};

const uint8_t MAX30102::INT_CFG_BIT[] = {
  7,      // FIFO Almost Full
  1,      // Temperature Ready
  6,      // PPG Ready
  5,      // ALC Overflow
  255     // Power Ready (không tồn tại)
};

const uint8_t MAX30102::INT_ST_REG[] = {
  0x00,   // FIFO Almost Full Status
  0x01,   // Temperature Ready Status
  0x00,   // PPG Ready Status
  0x00,   // ALC Overflow Status
  0x00    // Power Ready Status
};

const uint8_t MAX30102::INT_ST_BIT[] = {
  7,      // FIFO Almost Full
  1,      // Temperature Ready
  6,      // PPG Ready
  5,      // ALC Overflow
  0       // Power Ready
};

// ── Constructor ────────────────────────────────────────────────────────────
MAX30102::MAX30102(uint8_t addr, TwoWire& wire)
  : MAX3010xMultiLed(addr, wire) {}

// ── setDefaultConfiguration — gọi nội bộ bởi begin()/reset() ──────────────
bool MAX30102::setDefaultConfiguration() {
  MultiLedConfiguration cfg {};
  if (!setMultiLedConfiguration(cfg)) return false;
  if (!setLedCurrent(LED_RED, 90))    return false;
  if (!setLedCurrent(LED_IR,  80))    return false;
  if (!setResolution(RESOLUTION_18BIT_4110US))  return false;
  if (!setSamplingRate(SAMPLING_RATE_50SPS))     return false;
  if (!setSampleAveraging(SMP_AVE_NONE))         return false;
  if (!setADCRange(ADC_RANGE_16384NA))           return false;
  if (!enableFIFORollover())                     return false;
  if (!setMode(MODE_SPO2))                       return false;
  return true;
}

// ── setLedCurrent ──────────────────────────────────────────────────────────
bool MAX30102::setLedCurrent(MAX30102::Led led, uint8_t current) {
  return writeByte(LED_CFG_REG_BASE + static_cast<uint8_t>(led), current);
}

// ── setMultiLedConfiguration ───────────────────────────────────────────────
bool MAX30102::setMultiLedConfiguration(const MAX30102::MultiLedConfiguration& cfg) {
  uint8_t activeSlots = 0;
  for (int i = 0; i < 4; i++) {
    if (static_cast<uint8_t>(cfg.slot[i]) > 0b100) return false;
    if (cfg.slot[i] == SLOT_RED || cfg.slot[i] == SLOT_IR) {
      if (activeSlots != i) return false;
      activeSlots++;
    }
  }
  uint8_t rawCfg[2] = {};
  rawCfg[0] = static_cast<uint8_t>(cfg.slot[0]) | (static_cast<uint8_t>(cfg.slot[1]) << 4);
  rawCfg[1] = static_cast<uint8_t>(cfg.slot[2]) | (static_cast<uint8_t>(cfg.slot[3]) << 4);
  return setMultiLedConfigurationInternal(activeSlots, rawCfg);
}
