#pragma once
#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint8_t pin[4];        // valve pins: index 1..4 -> pin[0..3]
  bool    enableLog;
  Stream* log;           // default Serial if NULL and enableLog=true
} ValveCmd_Config;

// init GPIO + default all LOW
void ValveCmd_Begin(const ValveCmd_Config* cfg);

// call periodically in loop()
// supports:
//   VSET <idx 1..4> <0|1>
//   VALL <0|1>
void ValveCmd_Process(void);

// direct control
void ValveCmd_SetRaw(uint8_t index, uint8_t level);
void ValveCmd_SetAll(uint8_t level);

#ifdef __cplusplus
}
#endif
