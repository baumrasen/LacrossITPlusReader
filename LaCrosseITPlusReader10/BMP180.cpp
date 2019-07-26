#include "BMP180.h"

BMP180::BMP180() {
}


boolean BMP180::TryInitialize() {
  boolean result = false;

  Wire.begin();

  if (GetByte(0xD0) == 0x55) {
    m_ac1 = GetWord(0xAA);
    m_ac2 = GetWord(0xAC);
    m_ac3 = GetWord(0xAE);
    m_ac4 = GetWord(0xB0);
    m_ac5 = GetWord(0xB2);
    m_ac6 = GetWord(0xB4);

    m_b1 = GetWord(0xB6);
    m_b2 = GetWord(0xB8);

    m_mb = GetWord(0xBA);
    m_mc = GetWord(0xBC);
    m_md = GetWord(0xBE);

    result = true;
  }
  
  return result;
}

void BMP180::SetAltitudeAboveSeaLevel(int altitude) {
  m_altitudeAboveSeaLevel = altitude;
}

float BMP180::GetTemperature() {
  float result;

  SetByte(0xF4, 0x2E);
  delay(5);
  int ut = GetWord(0xF6);

  int b5 = CalculateB5(ut);
  result = (b5 + 8) >> 4;
  result /= 10;

  return result;
}

int BMP180::GetPressure() {
  int32_t ut, up, b3, b5, b6, x1, x2, x3, result;
  uint32_t b4, b7;

  // Get temperature
  SetByte(0xF4, 0x2E);
  delay(5);
  ut = GetWord(0xF6);

  // Measure with oversampling 2 -> needs 13.5ms
  SetByte(0xF4, 0xB4);
  delay(14);

  up = GetWord(0xF6);

  up <<= 8;
  up |= GetByte(0xF6 + 2);
  up >>= (6);

  b5 = CalculateB5(ut);

  b6 = b5 - 4000;
  x1 = ((int32_t)m_b2 * ((b6 * b6) >> 12)) >> 11;
  x2 = ((int32_t)m_ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = ((((int32_t)m_ac1 * 4 + x3) << 2) + 2) / 4;


  x1 = ((int32_t)m_ac3 * b6) >> 13;
  x2 = ((int32_t)m_b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = ((uint32_t)m_ac4 * (uint32_t)(x3 + 32768)) >> 15;
  b7 = ((uint32_t)up - b3) * (uint32_t)(50000UL >> 2);

  if (b7 < 0x80000000) {
    result = (b7 * 2) / b4;
  }
  else {
    result = (b7 / b4) * 2;
  }
  x1 = (result >> 8) * (result >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * result) >> 16;

  result += ((x1 + x2 + (int32_t)3791) >> 4);

  result /= powf(((float) 1.0 - ((float)m_altitudeAboveSeaLevel / 44330.0)), (float) 5.255);

  result /= 100.0;

  return result;
}

int BMP180::CalculateB5(int ut) {
  int x1 = (ut - (int32_t)m_ac6) * ((int32_t)m_ac5) >> 15;
  int x2 = ((int32_t)m_mc << 11) / (x1 + (int32_t)m_md);
  return x1 + x2;
}

byte BMP180::GetByte(byte address) {
  byte result;
  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(address); 
  Wire.endTransmission();

  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.requestFrom(BMP180_ADDRESS, 1);
  result = Wire.read();
  Wire.endTransmission(); 

  return result;
}

unsigned int BMP180::GetWord(byte address) {
  unsigned int result;

  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.requestFrom(BMP180_ADDRESS, 2);
  result = Wire.read();
  result <<= 8;
  result |= Wire.read();

  Wire.endTransmission();

  return result;
}

void BMP180::SetByte(byte address, byte data) {
  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(address); 
  Wire.write(data);  
  Wire.endTransmission();
}
