#ifndef _ADAFRUIT_AS7343_H
#define _ADAFRUIT_AS7343_H

#include <Arduino.h>
#include <Wire.h>

// -----------------------------
// Datasheet key registers (AS7343)
// -----------------------------
#define AS7343_I2CADDR_DEFAULT 0x39

#define AS7343_ENABLE   0x80
#define AS7343_ATIME    0x81
#define AS7343_WTIME    0x83

#define AS7343_STATUS2  0x90  // bit6 AVALID
#define AS7343_CFG1     0xC6  // AGAIN[4:0]
#define AS7343_ASTEP_L  0xD4
#define AS7343_ASTEP_H  0xD5
#define AS7343_CFG20    0xD6  // auto_smux [6:5]

#define AS7343_DATA_0_L 0x95  // DATA_0_L .. DATA_17_H are contiguous

// ---- Drift / offset compensation (Auto-Zero) ----
// AZ_CONFIG controls how often offset is re-zeroed to compensate temperature drift.
// CONTROL.SP_MAN_AZ can trigger a manual auto-zero (requires SP_EN=0 per datasheet).
#define AS7343_AZ_CONFIG   0xDE
#define AS7343_CONTROL     0xFA

// ENABLE bits
#define AS7343_ENABLE_PON    0x01  // bit0
#define AS7343_ENABLE_SP_EN  0x02  // bit1
#define AS7343_ENABLE_WEN    0x08  // bit3 (optional)

// STATUS2 bits
#define AS7343_STATUS2_AVALID 0x40 // bit6

// CONTROL bits
#define AS7343_CONTROL_SP_MAN_AZ 0x04 // bit2 (manual auto-zero)

// -----------------------------
// Gain enum: matches datasheet value table (0..12)
// 0:0.5x 1:1x 2:2x ... 9:256x 10:512x 11:1024x 12:2048x
// -----------------------------
typedef enum {
  AS7343_GAIN_0_5X  = 0,
  AS7343_GAIN_1X    = 1,
  AS7343_GAIN_2X    = 2,
  AS7343_GAIN_4X    = 3,
  AS7343_GAIN_8X    = 4,
  AS7343_GAIN_16X   = 5,
  AS7343_GAIN_32X   = 6,
  AS7343_GAIN_64X   = 7,
  AS7343_GAIN_128X  = 8,
  AS7343_GAIN_256X  = 9,
  AS7343_GAIN_512X  = 10,
  AS7343_GAIN_1024X = 11,
  AS7343_GAIN_2048X = 12,
} as7343_gain_t;

// Channel order (your “previous 12” order)
typedef enum {
  AS7343_CH_F1 = 0,
  AS7343_CH_F2,
  AS7343_CH_FZ,
  AS7343_CH_F3,
  AS7343_CH_F4,
  AS7343_CH_F5,
  AS7343_CH_FY,
  AS7343_CH_FXL,
  AS7343_CH_F6,
  AS7343_CH_F7,
  AS7343_CH_F8,
  AS7343_CH_NIR,
  AS7343_CH_COUNT = 12
} as7343_channel12_t;

class Adafruit_AS7341 {
public:
  Adafruit_AS7341();
  ~Adafruit_AS7341();

  bool begin(uint8_t i2c_addr = AS7343_I2CADDR_DEFAULT, TwoWire *wire = &Wire);

  // ---- Core configuration ----
  bool setASTEP(uint16_t astep_value);
  bool setATIME(uint8_t atime_value);
  bool setGain(as7343_gain_t gain_value);

  uint16_t getASTEP();
  uint8_t  getATIME();
  as7343_gain_t getGain();

  // Integration time:
  // tint = (ATIME+1)*(ASTEP+1)*2.78us  (datasheet)
  long  getTINT_ms();
  float toBasicCounts(uint16_t raw);

  // ---- SMUX auto sequencing ----
  // auto_smux = 2 => automatic 12ch (cycle1+cycle2)
  // auto_smux = 3 => automatic 18ch (cycle1+cycle2+cycle3)
  bool setAutoSMUX(uint8_t mode); // allowed: 0/2/3

  // ---- Read data ----
  bool readAllChannels(uint16_t *readings12);
  bool readRawDataRegs(uint16_t *data18);

  bool getIsDataReady();
  void powerEnable(bool enable_power);
  bool enableSpectralMeasurement(bool enable);

  // =========================================================
  // Drift / temperature offset compensation (Auto-Zero)
  // =========================================================

  // Set how often the device performs auto-zero (offset re-calibration)
  // AZ_NTH_ITERATION meanings (per datasheet):
  //   0   : never (NOT recommended)
  //   1   : every integration cycle
  //   2   : every 2 cycles
  //   ...
  //   255 : only before first measurement cycle (common default)
  bool setAutoZero(uint8_t az_nth_iteration);

  // Read back AZ_NTH_ITERATION
  uint8_t getAutoZero();

  // Trigger a manual auto-zero:
  // IMPORTANT: datasheet requires SP_EN = 0 before triggering manual AZ.
  // This function should:
  //   1) disableSpectralMeasurement(false)
  //   2) set CONTROL.SP_MAN_AZ
  //   3) optionally wait (typ. ~15ms) or poll if你实现了状态位
  //   4) restore SP_EN if desired (你可以在.cpp里做)
  bool manualAutoZero();

private:
  TwoWire  *_wire = nullptr;
  uint8_t   _addr = AS7343_I2CADDR_DEFAULT;
  uint8_t   _auto_smux = 2; // default as you requested

  // Low-level I2C helpers
  bool write8(uint8_t reg, uint8_t val);
  bool read8(uint8_t reg, uint8_t &val);
  bool readN(uint8_t start_reg, uint8_t *buf, size_t n);

  // Convert AGAIN enum to float ratio
  float gainToFloat(as7343_gain_t g);

  // Map DATA_0..DATA_17 into your 12-channel order.
  // Assumption (matches datasheet cycle listing order):
  // cycle1: [FZ,FY,FXL,NIR, VIS, FD] -> DATA_0..DATA_5
  // cycle2: [F2,F3,F4,F6,  VIS, FD] -> DATA_6..DATA_11
  // cycle3: [F1,F7,F8,F5,  VIS, FD] -> DATA_12..DATA_17
  void mapData18ToCh12(const uint16_t *data18, uint16_t *out12);
};

#endif
