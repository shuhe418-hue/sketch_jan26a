#pragma  
#include <Arduino.h>
#include <Wire.h>

#define I2C_HIGHDRIVER_ADRESS (0x79) 
#define I2C_DEVICEID          0x00
#define I2C_POWERMODE         0x01
#define I2C_FREQUENCY         0x02
#define I2C_SHAPE             0x03
#define I2C_BOOST             0x04
#define I2C_AUDIO             0x05
#define I2C_PVOLTAGE          0x06
#define I2C_P1VOLTAGE         0x06
#define I2C_P2VOLTAGE         0x07
#define I2C_P3VOLTAGE         0x08
#define I2C_P4VOLTAGE         0x09 N
#define I2C_UPDATEVOLTAGE     0x0A

extern boolean bPumpState[4];
extern uint8_t nPumpVoltageByte[4];
extern uint8_t nFrequencyByte;

void Highdriver4_init(void);
void Highdriver4_setvoltage(uint8_t _pump, uint8_t _voltageVpp);
void Highdriver4_setvoltage(void);
void Highdriver4_setfrequency(uint16_t _frequencyHz);

