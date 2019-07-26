#ifndef BMP180_H
#define BMP180_H

#include "Arduino.h"
#include "Wire.h"

#define BMP180_ADDRESS 0x77
           
class BMP180 {
public:
  BMP180();
 boolean TryInitialize();
 void SetAltitudeAboveSeaLevel(int altitude);
  float GetTemperature();
  int GetPressure();
  

private:
  int m_ac1, m_ac2, m_ac3, m_b1, m_b2, m_mb, m_mc, m_md;
  unsigned int m_ac4, m_ac5, m_ac6;
  int m_altitudeAboveSeaLevel = 0;
  int CalculateB5(int ut);
  byte GetByte(byte address);
  unsigned int GetWord(byte address);
  void SetByte(byte address, byte data);

  
};


#endif
