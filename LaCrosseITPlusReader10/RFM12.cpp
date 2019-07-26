#include "RFM12.h"


RFM12::RFM12(byte mosi, byte miso, byte sck, byte ss, byte irq) {
  m_mosi = mosi;
  m_miso = miso;
  m_sck = sck;
  m_ss = ss;
  m_irq = irq;

  m_debug = false;
  m_dataRate = 0x13;

  pinMode(m_mosi, OUTPUT);
  pinMode(m_miso, INPUT);
  pinMode(m_sck, OUTPUT);
  pinMode(m_ss, OUTPUT);
  pinMode(m_irq, INPUT);
}

void RFM12::SetDataRate(DataRates dataRate) {
  m_dataRate = dataRate;
  RFM12::spi(0xC600 | m_dataRate);
}

RFM12::DataRates RFM12::GetDataRate() {
  return (RFM12::DataRates)m_dataRate;
}

void RFM12::EnableReceiver(bool enable){
  if (enable) {
    spi(0x82C8);			// receiver on
    spi(0xCA81);			// set FIFO mode
    spi(0xCA83);			// enable FIFO

    // Ensure that the FIFO is empty
    for (byte i = 0; i < 50; i++) {
      spi(0xB000);
    }
  }
  else {
    // Receiver off 
    spi(0x8208);
  }
}

void RFM12::EnableTransmitter(bool enable){
  if (enable) {
    RFM12::spi(0x8238);
  }
  else {
    RFM12::spi(0x8208);
  }
}



void RFM12::SendByte(byte data) {
  RFM12::WaitForTransmitRegister();
  RFM12::spi(0xB800 | data);
}


byte RFM12::GetByteFromFifo() {
  return (byte)spi(0xB000);
}

#define clrb(pin) (*portOutputRegister(digitalPinToPort(pin)) &= ~digitalPinToBitMask(pin))
#define setb(pin) (*portOutputRegister(digitalPinToPort(pin)) |= digitalPinToBitMask(pin))
unsigned short RFM12::spi(unsigned short value) {
  volatile byte *misoPort = portInputRegister(digitalPinToPort(m_miso));
  byte misoBit = digitalPinToBitMask(m_miso);

  clrb(m_ss);
  for (byte i = 0; i < 16; i++) {
    if (value & 32768) {
      setb(m_mosi);
    }
    else {
      clrb(m_mosi);
    }
    value <<= 1;
    if (*misoPort & misoBit) {
      value |= 1;
    }
    setb(m_sck);
    asm("nop");
    asm("nop");
    clrb(m_sck);
  }
  setb(m_ss);
  return value;
}

void RFM12::WaitForTransmitRegister() {
  while (!(RFM12::spi(0x0000) & 0x8000)) {}
}

void RFM12::SendArray(byte *data, byte length) {
  if (m_debug) {
    Serial.print("Sending data: ");
    for (int p = 0; p < length; p++) {
      Serial.print(data[p], DEC);
      Serial.print(" ");
    }
    Serial.println();
  }

  // Transmitter on
  EnableTransmitter(true);

  // Sync, sync, sync ...
  RFM12::SendByte(0xAA);
  RFM12::SendByte(0xAA);
  RFM12::SendByte(0xAA);
  RFM12::SendByte(0x2D);
  RFM12::SendByte(0xD4);

  // Send the data
  for (int i = 0; i < length; i++) {
    RFM12::SendByte(data[i]);
  }

  // Transmitter off
  delay(1);
  EnableTransmitter(false);
}

bool RFM12::FifoHasData() {
  bool result = false;
  digitalWrite(m_ss, LOW);
  asm("nop");
  asm("nop");
  if (digitalRead(m_miso)) {
    result = true;
  }
  digitalWrite(m_ss, HIGH);

  return result;
}

byte RFM12::CalculateCRC(byte data[], int len) {
  int i, j;
  byte res = 0;
  for (j = 0; j < len; j++) {
    uint8_t val = data[j];
    for (i = 0; i < 8; i++) {
      uint8_t tmp = (uint8_t)((res ^ val) & 0x80);
      res <<= 1;
      if (0 != tmp) {
        res ^= 0x31;
      }
      val <<= 1;
    }
  }
  return res;
}

void RFM12::PowerDown(){
  spi(0x8201);
}

void RFM12::InitialzeLaCrosse() {
  // Deselect the RFM and wait until it is up
  digitalWrite(m_ss, HIGH);
  for (int i = 0; i < 10; i++) { delay(10); }

  RFM12::spi(0x8208);              // RX/TX off
  RFM12::spi(0x80E8);              // 80e8 CONFIGURATION EL,EF,868 band,12.5pF  (iT+ 915  80f8)
  RFM12::spi(0xA67c);              // FREQUENCY 868.300                         (a67c = 915.450 MHz)
  RFM12::spi(0xC600 | m_dataRate); // DATA RATE
  RFM12::spi(0xC26a);              // DATA FILTER
  RFM12::spi(0xCA12);              // FIFO AND RESET  8,SYNC,!ff,DR 
  RFM12::spi(0xCEd4);              // SYNCHRON PATTERN  0x2dd4 
  RFM12::spi(0xC49f);              // AFC during VDI HIGH +15 -15 AFC_control_commAND
  RFM12::spi(0x94a0);              // RECEIVER CONTROL VDI Medium 134khz LNA max DRRSI 103 dbm  
  RFM12::spi(0xCC77);              // 
  RFM12::spi(0x9850);              // Deviation 90 kHz 
  RFM12::spi(0xE000);              // 
  RFM12::spi(0xC800);              // 
  RFM12::spi(0xC040);              // 1.66MHz,2.2V 

  // Ensure that the FIFO is empty
  for (byte i = 0; i < 50; i++) {
    spi(0xB000);
  }

}

void RFM12::SetDebugMode(boolean mode) {
  m_debug = mode;
}

