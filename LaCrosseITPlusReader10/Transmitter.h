#ifndef _TRANSMITTER_h
#define _TRANSMITTER_h

#include "Arduino.h"
#include "RFM12.h"


class Transmitter {
 private:
   RFM12 *m_rfm;
   bool m_enabled;
   RFM12::DataRates m_dataRate;
   byte m_id;
   word m_interval;
   unsigned long m_newBatteryFlagResetTime;
   bool m_newBatteryFlag;
   float m_temperature;
   byte m_humidity;
   unsigned long m_lastTransmit;


 public:
   Transmitter(RFM12 *rfm);
   void Enable(bool enabled);
   bool Transmit();
   void SetParameters(byte id, word interval, bool newBatteryFlag, unsigned long newBatteryFlagResetTime, RFM12::DataRates dataRate);
   void SetValues(float temperature, byte humidity);
};

#endif

