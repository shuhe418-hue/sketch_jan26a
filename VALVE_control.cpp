#include "VALVE_control.h"
#include <stdio.h>

static ValveCmd_Config g_cfg;
static bool g_inited = false;

static inline Stream& LOG_() {
  if (g_cfg.log) return *g_cfg.log;
  return Serial;
}

static inline void logLine_(const char* s) {
  if (g_cfg.enableLog) LOG_().println(s);
}

void ValveCmd_Begin(const ValveCmd_Config* cfg) {
  if (!cfg) return;
  g_cfg = *cfg;

  if (g_cfg.enableLog && g_cfg.log == nullptr) {
    g_cfg.log = &Serial;
  }

  // --- 修改处：初始化为安全关闭状态 ---
  for (int i = 0; i < 4; i++) {
    digitalWrite(g_cfg.pin[i], LOW);   
    pinMode(g_cfg.pin[i], OUTPUT);     // Bartels 逻辑：输出低电平为关闭
  }

  g_inited = true;
  logLine_("[VALVE] ValveCmd ready (Bartels Mode: Default OFF)");
}

void ValveCmd_SetRaw(uint8_t index, uint8_t level) {
  if (!g_inited) return;
  if (index < 1 || index > 4) {
    logLine_("[VALVE] ERR invalid index");
    return;
  }
  if (!(level == 0 || level == 1)) {
    logLine_("[VALVE] ERR invalid level");
    return;
  }

  uint8_t pin = g_cfg.pin[index - 1];

  // --- 修改处：核心控制逻辑切换 ---
  if (level == 1) {
    // 开启：设置为输入模式，让引脚浮空 (High Impedance)
    pinMode(pin, INPUT); 
  } else {
    // 关闭：先写低电平，再设置为输出模式强行拉低
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }

  if (g_cfg.enableLog) {
    LOG_().print("[VALVE] VSET ");
    LOG_().print(index);
    LOG_().print(" ");
    LOG_().println(level ? "1 (FLOAT/OPEN)" : "0 (LOW/CLOSE)");
  }
}

void ValveCmd_SetAll(uint8_t level) {
  if (!g_inited) return;
  if (!(level == 0 || level == 1)) {
    logLine_("[VALVE] ERR invalid level");
    return;
  }

  // --- 修改处：批量控制逻辑切换 ---
  for (int i = 0; i < 4; i++) {
    uint8_t pin = g_cfg.pin[i];
    if (level == 1) {
      pinMode(pin, INPUT);
    } else {
      digitalWrite(pin, LOW);
      pinMode(pin, OUTPUT);
    }
  }

  if (g_cfg.enableLog) {
    LOG_().print("[VALVE] VALL ");
    LOG_().println(level ? "1 (ALL FLOAT)" : "0 (ALL LOW)");
  }
}

void ValveCmd_Process(void) {
  if (!g_inited) return;
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  int idx = 0, val = 0;

  if (sscanf(line.c_str(), "VSET %d %d", &idx, &val) == 2) {
    ValveCmd_SetRaw((uint8_t)idx, (uint8_t)val);
    return;
  }

  if (sscanf(line.c_str(), "VALL %d", &val) == 1) {
    ValveCmd_SetAll((uint8_t)val);
    return;
  }

  if (g_cfg.enableLog) {
    LOG_().print("[VALVE] ERR unknown cmd: ");
    LOG_().println(line);
  }
}