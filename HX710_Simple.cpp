#include "HX710_Simple.h"

HX710_Simple::HX710_Simple(uint8_t sckPin, uint8_t dtPin)
: _sck(sckPin), _dt(dtPin) {}

void HX710_Simple::Init_Hx710()
{
  pinMode(_sck, OUTPUT);
  pinMode(_dt, INPUT);
}

void HX710_Simple::Get_Maopi()
{
  HX710_Buffer = (long)HX710_Read();
  Weight_Maopi = HX710_Buffer / _rawDiv;
}

unsigned int HX710_Simple::Get_Weight()
{
  HX710_Buffer = (long)HX710_Read();
  HX710_Buffer = HX710_Buffer / _rawDiv;

  Weight_Shiwu = HX710_Buffer;
  Weight_Shiwu = Weight_Shiwu - Weight_Maopi; // 获取实物的AD采样数值。

  // 计算实物的实际重量
  // 你的原注释保留：每个传感器需要矫正这里的除数
  Weight_Shiwu = (unsigned int)((float)Weight_Shiwu / _calDiv);

  return (unsigned int)Weight_Shiwu;
}

//****************************************************
// 读取HX710（完全照抄你的 HX710_Read 逻辑，只把宏引脚换成 _dt/_sck）
//****************************************************
unsigned long HX710_Simple::HX710_Read(void) // 增益128
{
  unsigned long count;
  unsigned char i;
  bool Flag = 0;
  (void)Flag;

  digitalWrite(_dt, HIGH);
  delayMicroseconds(HX710_DELAY_US);
  digitalWrite(_sck, LOW);
  delayMicroseconds(HX710_DELAY_US);

  count = 0;
  while (digitalRead(_dt));

  // ===== 关键段：关中断，避免读 24bit 过程中被打断导致抖动 =====
  noInterrupts();

  for (i = 0; i < 24; i++)
  {
    digitalWrite(_sck, HIGH);
    delayMicroseconds(HX710_DELAY_US);

    count = count << 1;

    digitalWrite(_sck, LOW);
    delayMicroseconds(HX710_DELAY_US);

    if (digitalRead(_dt))
      count++;
  }

  digitalWrite(_sck, HIGH);
  count ^= 0x800000;
  delayMicroseconds(HX710_DELAY_US);

  digitalWrite(_sck, LOW);
  delayMicroseconds(HX710_DELAY_US);

  interrupts();
  // ================================================================

  return count;
}

// ===== 调参 =====
void HX710_Simple::setRawDiv(int div100)
{
  if (div100 <= 0) div100 = 1;
  _rawDiv = div100;
}

void HX710_Simple::setCalDiv(float calDiv)
{
  if (calDiv < 1e-6f) calDiv = 1e-6f;
  _calDiv = calDiv;
}

// ===== 滑窗滤波（对 Get_Weight 输出做均值）=====
void HX710_Simple::setFilterWindow(uint8_t win)
{
  if (win < 1) win = 1;
  if (win > MAX_WIN) win = MAX_WIN;
  _win = win;
  resetFilter();
}

void HX710_Simple::resetFilter()
{
  _cnt = 0;
  _idx = 0;
  _sum = 0;
  for (uint8_t i = 0; i < MAX_WIN; i++) _buf[i] = 0;
}

void HX710_Simple::_push(uint16_t v)
{
  if (_win == 1) {
    _cnt = 1;
    _idx = 0;
    _sum = v;
    _buf[0] = v;
    return;
  }

  if (_cnt < _win) {
    _buf[_idx] = v;
    _sum += v;
    _idx = (_idx + 1) % _win;
    _cnt++;
  } else {
    uint16_t old = _buf[_idx];
    _buf[_idx] = v;
    _sum += (uint32_t)v - (uint32_t)old;
    _idx = (_idx + 1) % _win;
  }
}

uint16_t HX710_Simple::_mean() const
{
  if (_cnt == 0) return 0;
  return (uint16_t)(_sum / _cnt);
}

unsigned int HX710_Simple::Get_WeightFiltered()
{
  unsigned int w = Get_Weight();
  _push((uint16_t)w);
  return (unsigned int)_mean();
}
