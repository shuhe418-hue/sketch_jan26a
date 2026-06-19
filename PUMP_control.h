#ifndef PUMP_CONTROL_H
#define PUMP_CONTROL_H

#include <Arduino.h>

/*
  PUMP control command processor (FINAL)

  Fixed Serial Command Protocol (DO NOT CHANGE):
    - PSET  <n> <0/1>      : set pump n (1..4) ON / OFF
    - PALL  <0/1>          : set all pumps ON / OFF
    - PFSET <Hz>           : set global pump frequency (shared by all pumps)
    - PVSET <n> <0..250>   : set pump n amplitude (Vpp, mapped to 0..31)
    - PVALL <0..250>       : set all pumps amplitude (Vpp, mapped to 0..31)

  Notes:
    - Frequency is global (hardware limitation of Highdriver4).
    - Pump ON/OFF is implemented by amplitude = 0 / non-zero.
    - PVSET/PVALL only set amplitude; actual output still depends on PSET/PALL state.
    - Voltage (amplitude) is handled inside Highdriver4 driver.

  Data flow:
    PC Serial
        ↓
    Main Control MCU
        ↓
    I2C (Highdriver4)
        ↓
    4 × Pump
*/

// 初始化模块（当前无参数，仅保留接口）
void PUMP_control_init(void);

// 处理一整行串口命令（不含 '\n'，内容会被 strtok 修改）
void PUMP_control_processLine(char *line);

#endif // PUMP_CONTROL_H
