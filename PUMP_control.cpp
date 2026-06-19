#include "PUMP_control.h"
#include <Wire.h>
#include <Arduino.h>

#include "HPD4_driver_test.h"

// =================================================
// 串口反馈（固定格式，供 PC 解析）
// =================================================
static void reply_ok(const String &msg) {
  Serial.print("OK ");
  Serial.println(msg);
}

static void reply_err(const String &msg) {
  Serial.print("ERR ");
  Serial.println(msg);
}

static bool parse_int(const char *s, long &out) {
  if (!s || !*s) return false;
  char *endp = nullptr;
  out = strtol(s, &endp, 10);
  return (endp && *endp == '\0');
}

// =================================================
// I2C apply —— 真实硬件动作
// =================================================
static void apply_pumps_i2c() {
  // [I2C_CMD]
  // 写入 0x06~0x0A，根据 bPumpState[] 更新 4 路泵输出
  Highdriver4_setvoltage();
}

// =================================================
// Public API
// =================================================
void PUMP_control_init(void) {
  // 当前 PSET / PALL / PFSET / PVSET / PVALL 不需要额外初始化
  // (Highdriver4_init() 建议在你的主程序 setup() 里调用一次)
}

void PUMP_control_processLine(char *line) {
  if (!line) return;

  // 去前导空白
  while (*line == ' ' || *line == '\t') line++;
  if (!*line) return;

  // 分词
  const int MAX_ARGS = 4;
  char *argv[MAX_ARGS] = {0};
  int argc = 0;

  char *tok = strtok(line, " \t");
  while (tok && argc < MAX_ARGS) {
    argv[argc++] = tok;
    tok = strtok(nullptr, " \t");
  }

  String cmd(argv[0]);
  cmd.toUpperCase();

  // =================================================
  // FIXED CMD: PSET <pump_id> <0/1>
  // =================================================
  if (cmd == "PSET") {
    if (argc != 3) {
      reply_err("PSET <1..4> <0/1>");
      return;
    }

    long pump = 0, state = 0;
    if (!parse_int(argv[1], pump) || !parse_int(argv[2], state)) {
      reply_err("PSET args must be integer");
      return;
    }

    if (pump < 1 || pump > 4 || (state != 0 && state != 1)) {
      reply_err("PSET range error");
      return;
    }

    bPumpState[pump - 1] = (state == 1);

    // [I2C_CMD]
    apply_pumps_i2c();

    reply_ok(String("PSET P") + pump + "=" + state);
    return;
  }

  // =================================================
  // FIXED CMD: PALL <0/1>
  // =================================================
  if (cmd == "PALL") {
    if (argc != 2) {
      reply_err("PALL <0/1>");
      return;
    }

    long state = 0;
    if (!parse_int(argv[1], state) || (state != 0 && state != 1)) {
      reply_err("PALL range error");
      return;
    }

    for (int i = 0; i < 4; i++) {
      bPumpState[i] = (state == 1);
    }

    // [I2C_CMD]
    apply_pumps_i2c();

    reply_ok(String("PALL=") + state);
    return;
  }

  // =================================================
  // FIXED CMD: PFSET <Hz>
  // =================================================
  if (cmd == "PFSET") {
    if (argc != 2) {
      reply_err("PFSET <Hz>");
      return;
    }

    long hz = 0;
    if (!parse_int(argv[1], hz)) {
      reply_err("PFSET arg must be integer");
      return;
    }

    // 负数直接拒绝；上限可以按你实际需求改小（例如 800 或 1000）
    if (hz < 0 || hz > 20000) {
      reply_err("PFSET range error");
      return;
    }

    // [I2C_CMD] 写 0x02 频率寄存器
    Highdriver4_setfrequency((uint16_t)hz);

    reply_ok(String("PFSET=") + hz);
    return;
  }

  // =================================================
  // NEW CMD: PVSET <pump_id> <Vpp>
  //   - set amplitude (voltage Vpp) for one pump
  //   - output still depends on bPumpState[] (OFF => written as 0)
  //   - Vpp range: 0..250 (mapped to 0..31)
  // =================================================
  if (cmd == "PVSET") {
    if (argc != 3) {
      reply_err("PVSET <1..4> <0..250>");
      return;
    }

    long pump = 0, vpp = 0;
    if (!parse_int(argv[1], pump) || !parse_int(argv[2], vpp)) {
      reply_err("PVSET args must be integer");
      return;
    }

    if (pump < 1 || pump > 4 || vpp < 0 || vpp > 250) {
      reply_err("PVSET range error");
      return;
    }

    // 这句会：更新 nPumpVoltageByte[pump-1] 并立刻写 I2C(0x06~0x0A)
    Highdriver4_setvoltage((uint8_t)pump, (uint8_t)vpp);

    reply_ok(String("PVSET P") + pump + "=" + vpp);
    return;
  }

  // =================================================
  // NEW CMD: PVALL <Vpp>
  //   - set amplitude (voltage Vpp) for all pumps
  // =================================================
  if (cmd == "PVALL") {
    if (argc != 2) {
      reply_err("PVALL <0..250>");
      return;
    }

    long vpp = 0;
    if (!parse_int(argv[1], vpp)) {
      reply_err("PVALL arg must be integer");
      return;
    }

    if (vpp < 0 || vpp > 250) {
      reply_err("PVALL range error");
      return;
    }

    // 只更新 4 路的 nPumpVoltageByte[]，再一次性 apply
    float temp = ((float)vpp) * 31.0f / 250.0f;
    uint8_t vb = (uint8_t)constrain((int)temp, 0, 31);

    for (int i = 0; i < 4; i++) {
      nPumpVoltageByte[i] = vb;
    }

    // [I2C_CMD]
    apply_pumps_i2c();

    reply_ok(String("PVALL=") + vpp);
    return;
  }

  // =================================================
  // Unknown command
  // =================================================
  reply_err(String("Unknown command: ") + argv[0]);
}
