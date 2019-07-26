#ifndef BMP180_H
#define BMP180_H

#include "Arduino.h"
#include "Wire.h"

#define BMP180_ADDRESS 0x77

class BMP180 {
public:
  BMP180();
  boolean TryInitialize();
  void SetAltitudeAboveSeaLevel(int32_t altitude);
  float GetTemperature(void);
  int32_t GetPressure(void);


private:
  int32_t m_altitudeAboveSeaLevel = 0;
  uint16_t GetRawTemperature(void);
  uint32_t GetRawPressure(void);
  int32_t CalculateB5(int32_t ut);
  uint8_t Read8(uint8_t addr);
  uint16_t Read16(uint8_t addr);
  void Write8(uint8_t addr, uint8_t data);

  int16_t m_ac1, m_ac2, m_ac3, m_b1, m_b2, m_mb, m_mc, m_md;
  uint16_t m_ac4, m_ac5, m_ac6;
};

#endif
