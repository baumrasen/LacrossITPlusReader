#ifndef _RFM12_h
#define _RFM12_h

#include "Arduino.h"

class RFM12 {
public:
  enum DataRates {
    DataRate17241 = 0x13,
    DataRate9579  = 0x23
  };
private:
  byte m_mosi, m_miso, m_sck, m_ss, m_irq;
  bool m_debug;
  byte m_dataRate;

  void WaitForTransmitRegister();
  void SendByte(byte data);
  
public:
  unsigned short spi(unsigned short value);
  RFM12(byte mosi, byte miso, byte sck, byte ss, byte irq);
  bool FifoHasData();
  void InitialzeLaCrosse();
  void SendArray(byte *data, byte length);
  byte GetByteFromFifo();
  void SetDataRate(DataRates dataRate);
  DataRates GetDataRate();
  void EnableReceiver(bool enable);
  void EnableTransmitter(bool enable);
  static byte CalculateCRC(byte data[], int len);
  void PowerDown();
  void SetDebugMode(boolean mode);
  
};


#endif

