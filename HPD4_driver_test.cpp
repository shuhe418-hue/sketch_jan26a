#include "HPD4_driver_test.h"

boolean bPumpState[4] = {false, false, false, false};
uint8_t nPumpVoltageByte[4] = {0x1F, 0x1F, 0x1F, 0x1F};
uint8_t nFrequencyByte = 0x40;

// 小工具：统一打印 I2C err
static inline void _printI2CErr(const char* tag, uint8_t err)
{
  Serial.print("[I2C] ");
  Serial.print(tag);
  Serial.print(" err=");
  Serial.println(err);
}

void Highdriver4_init(void) { // Initialize mp-Highdriver
  Wire.beginTransmission(I2C_HIGHDRIVER_ADRESS);

  Wire.write(I2C_POWERMODE);   // Start Register 0x01
  Wire.write(0x01);            // Register 0x01 = 0x01 (enable)
  Wire.write(nFrequencyByte);  // Register 0x02 = nFrequencyByte
  Wire.write(0x00);            // Register 0x03 = 0x00 (sine wave)
  Wire.write(0x00);            // Register 0x04 = 0x00 (800KHz)
  Wire.write(0x00);            // Register 0x05 = 0x00 (audio off)
  Wire.write(0x00);            // Register 0x06 = Amplitude1
  Wire.write(0x00);            // Register 0x07 = Amplitude2
  Wire.write(0x00);            // Register 0x08 = Amplitude3
  Wire.write(0x00);            // Register 0x09 = Amplitude4
  Wire.write(0x01);            // Register 0x0A = 0x01 (update)

  uint8_t err = Wire.endTransmission(true);
  _printI2CErr("init", err);

  // software-side state init
  bPumpState[0] = false;
  bPumpState[1] = false;
  bPumpState[2] = false;
  bPumpState[3] = false;

  nPumpVoltageByte[0] = 0x1F;
  nPumpVoltageByte[1] = 0x1F;
  nPumpVoltageByte[2] = 0x1F;
  nPumpVoltageByte[3] = 0x1F;
}

void Highdriver4_setvoltage(uint8_t _pump, uint8_t _voltageVpp) {
  float temp = (float)_voltageVpp;
  temp *= 31.0f;
  temp /= 250.0f;

  if (_pump >= 1 && _pump <= 4) {
    nPumpVoltageByte[_pump - 1] = constrain((int)temp, 0, 31);
  }

  Wire.beginTransmission(I2C_HIGHDRIVER_ADRESS);
  Wire.write(I2C_PVOLTAGE); // start register 0x06

  Wire.write((bPumpState[0] ? nPumpVoltageByte[0] : 0));
  Wire.write((bPumpState[1] ? nPumpVoltageByte[1] : 0));
  Wire.write((bPumpState[2] ? nPumpVoltageByte[2] : 0));
  Wire.write((bPumpState[3] ? nPumpVoltageByte[3] : 0));

  Wire.write(0x01); // update
  uint8_t err = Wire.endTransmission(true);

  // 打印：带上 pump 编号更好定位
  Serial.print("[I2C] setvoltage(p");
  Serial.print(_pump);
  Serial.print(") err=");
  Serial.println(err);
}

void Highdriver4_setvoltage(void) {
  Wire.beginTransmission(I2C_HIGHDRIVER_ADRESS);
  Wire.write(I2C_PVOLTAGE);

  Wire.write((bPumpState[0] ? nPumpVoltageByte[0] : 0));
  Wire.write((bPumpState[1] ? nPumpVoltageByte[1] : 0));
  Wire.write((bPumpState[2] ? nPumpVoltageByte[2] : 0));
  Wire.write((bPumpState[3] ? nPumpVoltageByte[3] : 0));

  Wire.write(0x01); // update
  uint8_t err = Wire.endTransmission(true);
  _printI2CErr("setvoltage(toggle)", err);
}

void Highdriver4_setfrequency(uint16_t _frequencyHz) {
  if (_frequencyHz >= 800) {
    nFrequencyByte = 0xFF;
  } else if (_frequencyHz >= 400) {
    _frequencyHz -= 400;
    _frequencyHz *= 64;
    _frequencyHz /= 400;
    nFrequencyByte = (uint8_t)(_frequencyHz | 0xC0);
  } else if (_frequencyHz >= 200) {
    _frequencyHz -= 200;
    _frequencyHz *= 64;
    _frequencyHz /= 200;
    nFrequencyByte = (uint8_t)(_frequencyHz | 0x80);
  } else if (_frequencyHz >= 100) {
    _frequencyHz -= 100;
    _frequencyHz *= 64;
    _frequencyHz /= 100;
    nFrequencyByte = (uint8_t)(_frequencyHz | 0x40);
  } else if (_frequencyHz >= 50) {
    _frequencyHz -= 50;
    _frequencyHz *= 64;
    _frequencyHz /= 50;
    nFrequencyByte = (uint8_t)(_frequencyHz | 0x00);
  } else {
    nFrequencyByte = 0x00;
  }

  Wire.beginTransmission(I2C_HIGHDRIVER_ADRESS);
  Wire.write(I2C_FREQUENCY);   // register 0x02
  Wire.write(nFrequencyByte);  // value
  uint8_t err = Wire.endTransmission(true);
  _printI2CErr("setfrequency", err);
}
