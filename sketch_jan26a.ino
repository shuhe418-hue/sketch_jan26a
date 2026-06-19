/*
  Main controller (ESP32-S3) - Dual I2C + Pumps/Valves + AS7343 streaming
  HX710 REMOVED (pressure sensors not connected)

  SparkFun AS7343 library version
  Serial protocol output:
    "Data AS7343: " + 18 raw channel slots (SparkFun original order)

  Raw 18-slot order:
    0:FZ   1:FY   2:FXL  3:NIR  4:VIS1  5:FD1
    6:F2   7:F3   8:F4   9:F6   10:VIS2 11:FD2
    12:F1  13:F7  14:F8  15:F5  16:VIS3 17:FD3
*/

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_AS7343.h>

// Original project drivers
#ince:\Bae:\BaiduNetdiskDownload\HX710.inoiduNetdiskDownload\HD710.inolude "HPD4_driver_test.h"
#include "PUMP_control.h"
#include "SERIAL_driver.h"
#include "VALVE_control.h"

// =================================================
// Pin / HW config (ESP32-S3)
// =================================================
static const uint32_t BAUD = 115200;

// I2C bus for Highdriver / Pumps
const int I2C1_SDA = 4;
const int I2C1_SCL = 5;

// I2C bus for AS7343 (Wire1)
const int I2C2_SDA = 8;
const int I2C2_SCL = 9;

// PWM LED (spectrum illumination)
const int SPECTRUM_LED_PIN = 15;
const int LED_FREQ = 1000;
const int LED_RES  = 8;  // duty: 0..255

// Output protocol
static const char* PREFIX = "Data AS7343:";

// AS7343 I2C address
static const uint8_t device_address = 0x39;

// Keep old compatibility label only
static uint8_t g_cycle_num = 18;

// AS7343 key registers (bank 0)
static const uint8_t REG_ATIME   = 0x81;
static const uint8_t REG_ASTEP_L = 0xD4;
static const uint8_t REG_ASTEP_H = 0xD5;

// =================================================
// Globals
// =================================================
static SerialDrv gSerial;
static ValveCmd_Config gValveCfg;
static int g_spectrum_led_duty = 0;

// SparkFun AS7343 instance on Wire1
static SfeAS7343ArdI2C gAs7343;
static uint16_t gRaw18[ksfAS7343NumChannels] = {0};

// User-configurable sensor params
static sfe_as7343_again_t g_again = AGAIN_64; // 默认先拉高
static int g_avg_reads = 4;                   // 平均次数
static uint32_t g_sample_period_ms = 200;

// 关键：恢复积分参数，默认 99/99 -> fullscale ≈ 10000
static uint8_t  g_atime = 99;
static uint16_t g_astep = 99;

// =================================================
// LED helpers
// =================================================
static inline void SpectrumLed_Init()
{
  ledcAttach(SPECTRUM_LED_PIN, LED_FREQ, LED_RES);
  ledcWrite(SPECTRUM_LED_PIN, 0);
}

static inline void SpectrumLed_SetDuty(int duty_0_255)
{
  if (duty_0_255 < 0) duty_0_255 = 0;
  if (duty_0_255 > 255) duty_0_255 = 255;
  g_spectrum_led_duty = duty_0_255;
  ledcWrite(SPECTRUM_LED_PIN, g_spectrum_led_duty);
}

// =================================================
// Low-level AS7343 register write helpers
// =================================================
static bool as7343WriteReg8(uint8_t reg, uint8_t value)
{
  Wire1.beginTransmission(device_address);
  Wire1.write(reg);
  Wire1.write(value);
  return (Wire1.endTransmission() == 0);
}

static bool as7343ReadReg8(uint8_t reg, uint8_t &value)
{
  return gAs7343.readRegisterBank(reg, value);
}

static bool applyAS7343Integration()
{
  // these registers are in bank 0; SparkFun driver exposes bank select
  if (!gAs7343.setRegisterBank(REG_BANK_0))
  {
    Serial.println(F("ERR: setRegisterBank(REG_BANK_0) failed"));
    return false;
  }

  // 为稳妥，改参数时先停测，再恢复
  gAs7343.enableSpectralMeasurement(false);
  delay(2);

  bool ok = true;
  ok &= as7343WriteReg8(REG_ATIME, g_atime);
  ok &= as7343WriteReg8(REG_ASTEP_L, (uint8_t)(g_astep & 0xFF));
  ok &= as7343WriteReg8(REG_ASTEP_H, (uint8_t)((g_astep >> 8) & 0xFF));

  delay(2);

  if (!gAs7343.enableSpectralMeasurement(true))
  {
    Serial.println(F("ERR: re-enable spectral measurement failed"));
    return false;
  }

  if (!ok)
  {
    Serial.println(F("ERR: writing ATIME/ASTEP registers failed"));
    return false;
  }

  // 回读确认
  uint8_t at = 0, al = 0, ah = 0;
  bool rd = true;
  rd &= as7343ReadReg8(REG_ATIME, at);
  rd &= as7343ReadReg8(REG_ASTEP_L, al);
  rd &= as7343ReadReg8(REG_ASTEP_H, ah);

  if (!rd)
  {
    Serial.println(F("WARN: readback of ATIME/ASTEP failed"));
  }
  else
  {
    uint16_t astep_rb = (uint16_t)al | ((uint16_t)ah << 8);
    Serial.printf("EVENT: INT set to ATIME=%u ASTEP=%u (readback ATIME=%u ASTEP=%u, fullscale=%lu)\n",
                  (unsigned)g_atime,
                  (unsigned)g_astep,
                  (unsigned)at,
                  (unsigned)astep_rb,
                  (unsigned long)((uint32_t)(at + 1) * (uint32_t)(astep_rb + 1)));
  }

  return true;
}

// =================================================
// Helpers
// =================================================
static void printSystemStatus()
{
  Serial.println(F("\n--- I2C Bus Scan (ESP32-S3 Dual Bus) ---"));

  Wire.beginTransmission(0x79);
  bool hd = (Wire.endTransmission() == 0);
  Serial.printf("[Wire  @12/13] Highdriver 0x79: %s\n", hd ? "ONLINE" : "OFFLINE");

  Wire1.beginTransmission(device_address);
  bool as = (Wire1.endTransmission() == 0);
  Serial.printf("[Wire1 @8/9]   AS7343     0x39: %s\n", as ? "ONLINE" : "OFFLINE");

  Serial.println(F("----------------------------------------\n"));
}

static uint8_t cycleToAutoSmux(uint8_t cycle_num)
{
  if (cycle_num == 12) return 2;
  if (cycle_num == 18) return 3;
  return 3;
}

static const char* againToStr(sfe_as7343_again_t g)
{
  switch (g)
  {
    case AGAIN_0_5:   return "0.5X";
    case AGAIN_1:     return "1X";
    case AGAIN_2:     return "2X";
    case AGAIN_4:     return "4X";
    case AGAIN_8:     return "8X";
    case AGAIN_16:    return "16X";
    case AGAIN_32:    return "32X";
    case AGAIN_64:    return "64X";
    case AGAIN_128:   return "128X";
    case AGAIN_256:   return "256X";
    case AGAIN_512:   return "512X";
    case AGAIN_1024:  return "1024X";
    case AGAIN_2048:  return "2048X";
    default:          return "UNKNOWN";
  }
}

static bool intToAgain(int g, sfe_as7343_again_t &out)
{
  switch (g)
  {
    case 0:    out = AGAIN_0_5;   return true; // 约定: 0 表示 0.5X
    case 1:    out = AGAIN_1;     return true;
    case 2:    out = AGAIN_2;     return true;
    case 4:    out = AGAIN_4;     return true;
    case 8:    out = AGAIN_8;     return true;
    case 16:   out = AGAIN_16;    return true;
    case 32:   out = AGAIN_32;    return true;
    case 64:   out = AGAIN_64;    return true;
    case 128:  out = AGAIN_128;   return true;
    case 256:  out = AGAIN_256;   return true;
    case 512:  out = AGAIN_512;   return true;
    case 1024: out = AGAIN_1024;  return true;
    case 2048: out = AGAIN_2048;  return true;
    default:   return false;
  }
}

static void printAS7343ParamLine()
{
  uint32_t fullscale = (uint32_t)(g_atime + 1) * (uint32_t)(g_astep + 1);
  Serial.printf("AS7343 PARAM: AUTOSMUX=%u GAIN=%s ATIME=%u ASTEP=%u FULLSCALE=%lu AVG=%d PERIOD_MS=%lu\n",
                (unsigned)cycleToAutoSmux(g_cycle_num),
                againToStr(g_again),
                (unsigned)g_atime,
                (unsigned)g_astep,
                (unsigned long)fullscale,
                g_avg_reads,
                (unsigned long)g_sample_period_ms);
}

static void printRaw18Label()
{
  Serial.println(F("AS7343 RAW18 ORDER:"));
  Serial.println(F("0:FZ 1:FY 2:FXL 3:NIR 4:VIS1 5:FD1 6:F2 7:F3 8:F4 9:F6 10:VIS2 11:FD2 12:F1 13:F7 14:F8 15:F5 16:VIS3 17:FD3"));
}

static bool applyAS7343Gain()
{
  if (!gAs7343.setAgain(g_again))
  {
    Serial.println(F("ERR: setAgain failed"));
    return false;
  }
  Serial.printf("EVENT: GAIN set to %s\n", againToStr(g_again));
  return true;
}

static bool readAS7343_Avg(uint16_t out18[18], int avg_reads)
{
  if (avg_reads < 1) avg_reads = 1;

  uint32_t acc[18] = {0};
  int goodReads = 0;

  for (int n = 0; n < avg_reads; ++n)
  {
    uint32_t t0 = millis();
    while (!gAs7343.getSpectralValidStatus())
    {
      if (millis() - t0 > 30) break;
      delay(1);
    }

    if (!gAs7343.readSpectraDataFromSensor())
    {
      delay(2);
      continue;
    }

    uint16_t tmp[ksfAS7343NumChannels] = {0};
    int channelsRead = gAs7343.getData(tmp);
    if (channelsRead < 18)
    {
      delay(2);
      continue;
    }

    for (int i = 0; i < 18; ++i)
    {
      acc[i] += tmp[i];
    }
    goodReads++;
    delay(2);
  }

  if (goodReads <= 0) return false;

  for (int i = 0; i < 18; ++i)
  {
    out18[i] = (uint16_t)(acc[i] / goodReads);
  }
  return true;
}

// =================================================
// Command handlers
// =================================================
static bool tryHandleSpectrumCmd(char* line)
{
  char* endp = nullptr;
  long v = strtol(line, &endp, 10);
  if (endp != line && *endp == '\0')
  {
    SpectrumLed_SetDuty((int)v);
    Serial.printf("EVENT: LED set to %d\n", g_spectrum_led_duty);
    return true;
  }

  if (strncasecmp(line, "AMB", 3) == 0 || strncasecmp(line, "WB", 2) == 0)
  {
    Serial.println("WARN: AMB/WB commands are not supported in this AS7343 mode.");
    return true;
  }

  return false;
}

static bool tryHandleValveCmd(char* line)
{
  int idx = 0, val = 0;
  if (sscanf(line, "VSET %d %d", &idx, &val) == 2)
  {
    ValveCmd_SetRaw((uint8_t)idx, (uint8_t)val);
    return true;
  }
  if (sscanf(line, "VALL %d", &val) == 1)
  {
    ValveCmd_SetAll((uint8_t)val);
    return true;
  }
  return false;
}

static bool tryHandleAS7343ParamCmd(char* line)
{
  if (strcasecmp(line, "SP?") == 0)
  {
    printAS7343ParamLine();
    printRaw18Label();
    return true;
  }

  int g = -1;
  if (sscanf(line, "GAIN %d", &g) == 1)
  {
    sfe_as7343_again_t againTmp;
    if (!intToAgain(g, againTmp))
    {
      Serial.println("ERR: supported GAIN values are 0 1 2 4 8 16 32 64 128 256 512 1024 2048 (0 means 0.5X)");
      return true;
    }

    g_again = againTmp;
    applyAS7343Gain();
    return true;
  }

  int at = -1;
  if (sscanf(line, "ATIME %d", &at) == 1)
  {
    if (at < 0 || at > 255)
    {
      Serial.println("ERR: ATIME must be 0..255");
      return true;
    }
    g_atime = (uint8_t)at;
    applyAS7343Integration();
    return true;
  }

  int as = -1;
  if (sscanf(line, "ASTEP %d", &as) == 1)
  {
    if (as < 0 || as > 65535)
    {
      Serial.println("ERR: ASTEP must be 0..65535");
      return true;
    }
    g_astep = (uint16_t)as;
    applyAS7343Integration();
    return true;
  }

  int at2 = -1, as2 = -1;
  if (sscanf(line, "INT %d %d", &at2, &as2) == 2)
  {
    if (at2 < 0 || at2 > 255)
    {
      Serial.println("ERR: ATIME must be 0..255");
      return true;
    }
    if (as2 < 0 || as2 > 65535)
    {
      Serial.println("ERR: ASTEP must be 0..65535");
      return true;
    }

    g_atime = (uint8_t)at2;
    g_astep = (uint16_t)as2;
    applyAS7343Integration();
    return true;
  }

  int avgN = -1;
  if (sscanf(line, "AVG %d", &avgN) == 1)
  {
    if (avgN < 1) avgN = 1;
    if (avgN > 16) avgN = 16;
    g_avg_reads = avgN;
    Serial.printf("EVENT: AVG set to %d\n", g_avg_reads);
    return true;
  }

  int per = -1;
  if (sscanf(line, "PERIOD %d", &per) == 1)
  {
    if (per < 20) per = 20;
    if (per > 5000) per = 5000;
    g_sample_period_ms = (uint32_t)per;
    Serial.printf("EVENT: PERIOD set to %lu ms\n", (unsigned long)g_sample_period_ms);
    return true;
  }

  return false;
}

static void onSerialLine(const char* line, void*)
{
  if (!line) return;

  static char buf[128];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  if (tryHandleSpectrumCmd(buf)) return;
  if (tryHandleValveCmd(buf)) return;
  if (tryHandleAS7343ParamCmd(buf)) return;

  int idx = 0, vpp = 0;
  if (sscanf(buf, "PVSET %d %d", &idx, &vpp) == 2)
  {
    Highdriver4_setvoltage((uint8_t)idx, (uint8_t)vpp);
    Serial.printf("EVENT: PVSET %d %d\n", idx, vpp);
    return;
  }

  PUMP_control_processLine(buf);
}

static void onSerialErr(const char* err, void*)
{
  Serial.printf("SERIAL_ERR: %s\n", err ? err : "unknown");
}

// =================================================
// setup / loop
// =================================================
void setup()
{
  Serial.begin(BAUD);
  delay(500);

  Wire.begin(I2C1_SDA, I2C1_SCL, 100000);
  Wire1.begin(I2C2_SDA, I2C2_SCL, 400000);
  Wire1.setClock(400000);

  printSystemStatus();

  SpectrumLed_Init();

  if (!gAs7343.begin(device_address, Wire1))
  {
    Serial.println(F("CRITICAL: gAs7343.begin() failed!"));
  }
  else if (!gAs7343.powerOn())
  {
    Serial.println(F("CRITICAL: gAs7343.powerOn() failed!"));
  }
  else if (!gAs7343.setAutoSmux(AUTOSMUX_18_CHANNELS))
  {
    Serial.println(F("CRITICAL: setAutoSmux(AUTOSMUX_18_CHANNELS) failed!"));
  }
  else if (!applyAS7343Gain())
  {
    Serial.println(F("CRITICAL: applyAS7343Gain() failed!"));
  }
  else if (!gAs7343.enableSpectralMeasurement())
  {
    Serial.println(F("CRITICAL: enableSpectralMeasurement() failed!"));
  }
  else if (!applyAS7343Integration())
  {
    Serial.println(F("CRITICAL: applyAS7343Integration() failed!"));
  }
  else
  {
    Serial.println(F("SUCCESS: AS7343 started (SparkFun lib + raw INT registers)."));
    printAS7343ParamLine();
    printRaw18Label();
  }

  // Pumps
  Highdriver4_setfrequency(100);
  Highdriver4_init();
  Highdriver4_setvoltage();
  PUMP_control_init();

  // Valve init
  gValveCfg.pin[0] = 40;
  gValveCfg.pin[1] = 39;
  gValveCfg.pin[2] = 38;
  gValveCfg.pin[3] = 37;
  gValveCfg.enableLog = true;
  gValveCfg.log = &Serial;
  ValveCmd_Begin(&gValveCfg);

  // Serial driver
  SerialDrvConfig sCfg;
  sCfg.baud = BAUD;
  sCfg.rx_max_len = 96;
  sCfg.echo_rx = true;
  sCfg.trim_space = true;
  gSerial.begin(Serial, sCfg);
  gSerial.onLine(onSerialLine);
  gSerial.onError(onSerialErr);

  Serial.println(F(">>> SYSTEM READY <<<"));
}

void loop()
{
  gSerial.poll();

  static uint32_t lastSpectrumTime = 0;
  if (millis() - lastSpectrumTime >= g_sample_period_ms)
  {
    lastSpectrumTime = millis();

    bool ok = readAS7343_Avg(gRaw18, g_avg_reads);

    if (ok)
    {
      Serial.print(PREFIX);
      Serial.print(' ');

      for (int i = 0; i < 18; i++)
      {
        Serial.print(gRaw18[i]);
        if (i < 17) Serial.print(' ');
      }

      Serial.println();
    }
    else
    {
      Serial.print(PREFIX);
      Serial.println(" -1");
    }
  }
}