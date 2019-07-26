#include "WH25.h"

/*
  WH25  17.241 kbps  868.3 MHz
  -------------------------------
  Extracted data:
            ?I IT TT HH PP PP CC BB
   aa 2d d4 e5 02 72 28 27 21 c9 bb aa
    0  1  2  3  4  5  6  7  8  9 10
             0  1  2  3  4  5  6  7
   II = Sensor ID (based on 2 different sensors). Does not change at battery change.
   T TT = Temperature (+40*10)
   HH = Humidity
   PP PP = Pressure (*10)
   CC = Checksum of previous 6 bytes (binary sum truncated to 8 bit)
   BB = Bitsum (XOR) of the 6 data bytes (high and low nibble exchanged)
   
   OK VALUES WH25 50 Header=14,Temperature=xx.xx,Humidity=69,Pressure=1013.80, [E3 2A 8D 45 27 9A A0 AA AA AA 00 7D]
   OK VALUES WH25 43 Header=14,Temperature=23.80,Humidity=56,Pressure=1012.10, [E2 B2 7E 38 27 89 FA AA AA AA 00 5F]

*/

void WH25::DecodeFrame(byte *bytes, struct Frame *frame) {

  uint8_t tempkorr = 0;
  frame->IsValid = true;
  frame->ID = 0;
  frame->NewBatteryFlag = false;
  frame->LowBatteryFlag = false;
  frame->ErrorFlag = false;

  frame->HasTemperature = true;
  frame->HasHumidity = true;
  frame->HasRain = false;
  frame->HasWindSpeed = false;
  frame->HasWindDirection = false;
  frame->HasWindGust = false;
  frame->HasPressure = true;
  frame->HasUV = false;
  frame->HasstrikesTotal = false;
  frame->HasstrikesDistance = false;


  frame->Header = bytes[0] >> 4;
  if (frame->Header == 0xE) {
    frame->IsValid = true;
  }
  else {
    frame->IsValid = false;
  }

  if (frame->IsValid) {
    // Verify checksum
    uint8_t checksum = 0, bitsum = 0;

    for (size_t n = 0; n <= 5; ++n) {
      checksum += bytes[n];
      bitsum ^= bytes[n];
    }

    bitsum = (bitsum << 4) | (bitsum >> 4);     // Swap nibbles

    if (checksum == bytes[6] & bytes[9] == 0xAA ) {
      tempkorr = 0;
      frame->IsValid = true;
    } else {
      frame->IsValid = false;
    }

    if (frame->IsValid & bitsum == bytes[7]) { // WH25A ab Release 20/14 andere Temp-Darstellung
      //if (checksum != bytes[6] || bitsum != bytes[7]) & ( bytes[9] != 0xAA) {
      tempkorr = 1;
    } else {
      tempkorr = 0;
    }
  }

  if (frame->IsValid) {
    frame->ID = (bytes[0] << 4) | (bytes[1] >> 4);

    frame->NewBatteryFlag = false;
    frame->ErrorFlag = false;
    frame->LowBatteryFlag = true;
    frame->LowBatteryFlag = (bytes[1] & 0x08) >> 3;
    
    
    // Temperature (�C)
    int temp = (bytes[1] & 0x07) << 8 | bytes[2]; // 0x7ff if invalid
    
   /* int temp = ((bytes[1] & 0xF) << 8) | bytes[2];
    if (tempkorr == 1) {
      temp = ((bytes[1] & 0x2) << 8) | bytes[2]; // ab WH25A 20/14 ist das Protokoll anders !?
    }
    */
    frame->Temperature = ((temp * 0.1) - 40.0);  // range -40.0-60.0 C

    // Humidity (%rH)
    frame->Humidity = bytes[3];

    frame->Pressure = ((bytes[4] << 8) | bytes[5]) * 0.1;

  }
}


String WH25::AnalyzeFrame(byte *data) {
  struct Frame frame;
  DecodeFrame(data, &frame);

  byte frameLength = WH25::FRAME_LENGTH;

  return WSBase::AnalyzeFrame(data, &frame, frameLength, "WH25");
}

String WH25::GetFhemDataString(byte *data) {
  String fhemString2 = "";
  // if ((data[0] >> 4) == 0x0E) {
  struct Frame frame2;
  DecodeFrame(data, &frame2);
  if (frame2.IsValid) {
    //fhemString2 = BuildFhemDataString(&frame2, 5);
    fhemString2 = BuildKVDataString(&frame2, 5);
  }
  //}

  return fhemString2;
}

bool WH25::TryHandleData(byte *data) {
  String fhemString = GetFhemDataString(data);

  if (fhemString.length() > 0) {
    Serial.println(fhemString);
  }

  return fhemString.length() > 0;
}



bool WH25::IsValidDataRate(unsigned long dataRate) {
  return dataRate == 17241ul;
}
