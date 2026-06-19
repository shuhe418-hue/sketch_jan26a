#include "Arduino.h"
#include <Wire.h>

#include "Adafruit_AS7343.h"

// ====== ADD: if your .h doesn't define it yet, keep this here ======
#ifndef AS7343_AZ_CONFIG
#define AS7343_AZ_CONFIG 0xDE
#endif
// ===================================================================

// 推荐的 Auto-Zero 周期：每 8 次积分做一次 offset re-zero（抗温漂/漂移）
static const uint8_t AS7343_AZ_NTH_DEFAULT = 8;

Adafruit_AS7341::Adafruit_AS7341() {}
Adafruit_AS7341::~Adafruit_AS7341() {}

bool Adafruit_AS7341::begin(uint8_t i2c_addr, TwoWire *wire) {
  _addr = i2c_addr;
  _wire = wire;

  // NOTE: Wire.begin(...) should be done in the .ino (ESP32 needs SDA/SCL pins)
  // so we do not call it here.

  // Power on
  powerEnable(true);

  // ---------------------------------------------------------
  // ADD: Auto-Zero for drift/temperature offset compensation
  // Datasheet: AZ_CONFIG(0xDE) controls how often auto-zero happens.
  // 0 = never (not recommended), 255 = only before first cycle.
  // Here we choose 8 as a balanced value for long-running stability.
  // ---------------------------------------------------------
  // best practice: ensure spectral engine is disabled while changing config
  enableSpectralMeasurement(false);
  write8(AS7343_AZ_CONFIG, AS7343_AZ_NTH_DEFAULT);

  // Default to scheme A auto_smux=2 (adjustable by setAutoSMUX)
  setAutoSMUX(_auto_smux);

  return true;
}

bool Adafruit_AS7341::write8(uint8_t reg, uint8_t val) {
  if (!_wire) return false;
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->write(val);
  return (_wire->endTransmission(true) == 0);
}

bool Adafruit_AS7341::read8(uint8_t reg, uint8_t &val) {
  if (!_wire) return false;
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) return false;

  if (_wire->requestFrom((int)_addr, 1, (int)true) != 1) return false;
  val = _wire->read();
  return true;
}

bool Adafruit_AS7341::readN(uint8_t start_reg, uint8_t *buf, size_t n) {
  if (!_wire || !buf || n == 0) return false;

  _wire->beginTransmission(_addr);
  _wire->write(start_reg);
  if (_wire->endTransmission(false) != 0) return false;

  size_t got = _wire->requestFrom((int)_addr, (int)n, (int)true);
  if (got != n) return false;

  for (size_t i = 0; i < n; i++) buf[i] = _wire->read();
  return true;
}

void Adafruit_AS7341::powerEnable(bool enable_power) {
  uint8_t en = 0;
  read8(AS7343_ENABLE, en);
  if (enable_power) en |= AS7343_ENABLE_PON;
  else              en &= ~AS7343_ENABLE_PON;
  write8(AS7343_ENABLE, en);
}

bool Adafruit_AS7341::enableSpectralMeasurement(bool enable) {
  uint8_t en = 0;
  if (!read8(AS7343_ENABLE, en)) return false;

  if (enable) en |= AS7343_ENABLE_SP_EN;
  else        en &= ~AS7343_ENABLE_SP_EN;

  return write8(AS7343_ENABLE, en);
}

bool Adafruit_AS7341::getIsDataReady() {
  uint8_t s2 = 0;
  if (!read8(AS7343_STATUS2, s2)) return false;
  return (s2 & AS7343_STATUS2_AVALID) != 0;
}

bool Adafruit_AS7341::setATIME(uint8_t atime_value) {
  return write8(AS7343_ATIME, atime_value);
}

uint8_t Adafruit_AS7341::getATIME() {
  uint8_t v = 0;
  read8(AS7343_ATIME, v);
  return v;
}

bool Adafruit_AS7341::setASTEP(uint16_t astep_value) {
  bool ok1 = write8(AS7343_ASTEP_L, (uint8_t)(astep_value & 0xFF));
  bool ok2 = write8(AS7343_ASTEP_H, (uint8_t)(astep_value >> 8));
  return ok1 && ok2;
}

uint16_t Adafruit_AS7341::getASTEP() {
  uint8_t lo = 0, hi = 0;
  read8(AS7343_ASTEP_L, lo);
  read8(AS7343_ASTEP_H, hi);
  return (uint16_t)lo | ((uint16_t)hi << 8);
}

bool Adafruit_AS7341::setGain(as7343_gain_t gain_value) {
  uint8_t g = (uint8_t)gain_value & 0x1F;
  return write8(AS7343_CFG1, g);
}

as7343_gain_t Adafruit_AS7341::getGain() {
  uint8_t g = 0;
  read8(AS7343_CFG1, g);
  g &= 0x1F;
  if (g > 12) g = 12;
  return (as7343_gain_t)g;
}

bool Adafruit_AS7341::setAutoSMUX(uint8_t mode) {
  if (!(mode == 0 || mode == 2 || mode == 3)) return false;

  enableSpectralMeasurement(false);

  uint8_t v = 0;
  if (!read8(AS7343_CFG20, v)) return false;
  v &= ~(0x3 << 5);
  v |= ((mode & 0x3) << 5);
  if (!write8(AS7343_CFG20, v)) return false;

  _auto_smux = mode;
  return true;
}

long Adafruit_AS7341::getTINT_ms() {
  uint16_t astep = getASTEP();
  uint8_t  atime = getATIME();
  float us = (float)(atime + 1) * (float)(astep + 1) * 2.78f;
  return (long)(us / 1000.0f);
}

float Adafruit_AS7341::gainToFloat(as7343_gain_t g) {
  switch (g) {
    case AS7343_GAIN_0_5X:  return 0.5f;
    case AS7343_GAIN_1X:    return 1.0f;
    case AS7343_GAIN_2X:    return 2.0f;
    case AS7343_GAIN_4X:    return 4.0f;
    case AS7343_GAIN_8X:    return 8.0f;
    case AS7343_GAIN_16X:   return 16.0f;
    case AS7343_GAIN_32X:   return 32.0f;
    case AS7343_GAIN_64X:   return 64.0f;
    case AS7343_GAIN_128X:  return 128.0f;
    case AS7343_GAIN_256X:  return 256.0f;
    case AS7343_GAIN_512X:  return 512.0f;
    case AS7343_GAIN_1024X: return 1024.0f;
    case AS7343_GAIN_2048X: return 2048.0f;
    default:                return 1.0f;
  }
}

float Adafruit_AS7341::toBasicCounts(uint16_t raw) {
  float gain = gainToFloat(getGain());

  uint16_t astep = getASTEP();
  uint8_t  atime = getATIME();
  float tint_ms = (float)(atime + 1) * (float)(astep + 1) * 2.78f / 1000.0f;

  if (tint_ms <= 0.0f) return 0.0f;
  return (float)raw / (gain * tint_ms);
}

bool Adafruit_AS7341::readRawDataRegs(uint16_t *data18) {
  if (!data18) return false;

  enableSpectralMeasurement(true);

  while (!getIsDataReady()) {
    delay(1);
  }

  uint8_t buf[36] = {0};
  if (!readN(AS7343_DATA_0_L, buf, sizeof(buf))) return false;

  for (int i = 0; i < 18; i++) {
    uint8_t lo = buf[i * 2 + 0];
    uint8_t hi = buf[i * 2 + 1];
    data18[i] = (uint16_t)lo | ((uint16_t)hi << 8);
  }
  return true;
}

void Adafruit_AS7341::mapData18ToCh12(const uint16_t *d, uint16_t *o) {
  for (int i = 0; i < 12; i++) o[i] = 0;

  if (_auto_smux == 2) {
    o[0]  = 0;
    o[1]  = d[6];
    o[2]  = d[0];
    o[3]  = d[7];
    o[4]  = d[8];
    o[5]  = 0;
    o[6]  = d[1];
    o[7]  = d[2];
    o[8]  = d[9];
    o[9]  = 0;
    o[10] = 0;
    o[11] = d[3];
    return;
  }

  if (_auto_smux == 3) {
    o[0]  = d[12];
    o[1]  = d[6];
    o[2]  = d[0];
    o[3]  = d[7];
    o[4]  = d[8];
    o[5]  = d[15];
    o[6]  = d[1];
    o[7]  = d[2];
    o[8]  = d[9];
    o[9]  = d[13];
    o[10] = d[14];
    o[11] = d[3];
    return;
  }
}

bool Adafruit_AS7341::readAllChannels(uint16_t *readings12) {
  if (!readings12) return false;

  uint16_t d18[18] = {0};
  if (!readRawDataRegs(d18)) return false;

  mapData18ToCh12(d18, readings12);
  return true;
}
