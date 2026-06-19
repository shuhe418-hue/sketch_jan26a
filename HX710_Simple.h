#pragma once
#include <Arduino.h>

class HX710_Simple {
public:
  // 构造：给每个 HX710 一套引脚
  HX710_Simple(uint8_t sckPin, uint8_t dtPin);

  // ====== 完全照抄你的函数语义 ======
  void Init_Hx710();        // 初始化
  void Get_Maopi();         // 清零/去皮（保存 Weight_Maopi）
  unsigned int Get_Weight();// 读取并计算重量（返回 unsigned int）
  unsigned long HX710_Read(void); // 读 24-bit（增益128）

  // ====== 额外：不影响原函数，用于调参 ======
  void setRawDiv(int div100);     // 你代码里 HX710_Buffer/100 这一步（默认100）
  void setCalDiv(float calDiv);   // 你代码里 /7.35 这个除数（默认7.35）

  // ====== 额外：滑窗滤波（对 Get_Weight 的输出做均值）=====
  void setFilterWindow(uint8_t win);       // 1..32，默认1=不滤波
  unsigned int Get_WeightFiltered();        // 滤波后的 weight
  void resetFilter();                       // 清空滤波缓存

  // 调试：读出内部去皮值
  long getMaopi() const { return Weight_Maopi; }

private:
  // 引脚
  uint8_t _sck;
  uint8_t _dt;

  // 你原来的全局变量（每个对象一份）
  long HX710_Buffer = 0;
  long Weight_Maopi = 0;
  long Weight_Shiwu = 0;

  // 你原代码里的两个“魔数”
  int   _rawDiv = 100;     // HX710_Buffer/100
  float _calDiv = 7.35f;   // (float)Weight_Shiwu/7.35

  // 滑窗滤波：对 weight 输出做均值
  static const uint8_t MAX_WIN = 32;
  uint8_t _win = 1;
  uint8_t _cnt = 0;
  uint8_t _idx = 0;
  uint32_t _sum = 0;
  uint16_t _buf[MAX_WIN] = {0};

private:
  void _push(uint16_t v);
  uint16_t _mean() const;

  // 你原始时序：delayMicroseconds(1)
  // 需要更稳就改成 2 或 3
  static const uint8_t HX710_DELAY_US = 1;
};
