#include "W136.h"

// The following information can be received:
// Humidity, temperature, wind direction, wind average, wind gust, rainfall, UV and lightning.
// That is more than te maximum of 4 values per device for espeasy. The plugins functionality is therefore
// divided per sensorgroup.

// The plugin can (and should) be used more then one time, however only one plugin instance can be
// the main plugin. The plugin function can be selected by a dropdown and only the MAIN plugin has
// the ability to set the sensor address (remote unit ID).

// To find out the current ID of your Ventus remote unit, simply setup a task with instance type "MAIN Temp/Hygro" and observe the LOG.
// Everytime the RFM69 receives a valid packet from a Ventus W266 it shows up in the LOG (approx. every 30 seconds)
// The first number after the "RX:" is the ID of the unit received.
// This ID-number then has to be entered in the "Unit ID" field in the task setup of the "MAIN Temp/Hygro" instance.
// Data is sent to the specified server as soon as it is received.

// RFM69 RX-buffer content:
// ************************
// IDhh 1A tlth ?b tlth wb alahglgh rlrh?? uv ld?? lllhcrc
// 9827 1A B100 00 B100 06 00000000 1E0000 00 3F8A 2A0017
//  0 1  2  3 4  5  6 7  8  9 0 1 2  3 4 5  6  7 8  9 0 1
//
// ID ..... ID of the remote unit. Changes randomly each time the batteries are removed. Is needed to identify "your" unit.
// hh ..... humidity -> Humidity bcd encoded
// tlth ... temperature (low/high byte) > Temperature is stored as a 16bit integer holding the temperature in degree celcius*10
// b ...... battery low > This uint8 is 00 when battery is ok, 01 when battery is low
// wb ..... bearing (cw 0-15) > The wind bearing in 16 clockwise steps (0 = north, 4 = east, 8 = south and C = west)
// alah ... wind average (low/high byte) > A uint16 holding the wind avarage in m/s
// glgh ... wind gust (low/high byte) > A uint16 holding the wind gust in m/s
// rlrh ... rainfall (low/high byte) > Accumulated rainfall in 1/4mm
// uv ..... uv-index > The UV value * 10
// ld ..... lightningstorm-distance (3F max) > The distance to the stormfront in km
// lllh ... strike count (low/high byte) > A uint16 holding the accumulated number of detected lightning strikes
// crc .... CRC checksum > Poly 0x31, init 0xff, revin&revout, xorout 0x00. Like Maxim 1-wire but with a 0xff init value

// If you got any questions, send me an email to: huawatuam@gmail.com
//pejonp 18.1.2018

byte W136::CalculateCRC(byte data[]) {
  uint8_t crc = 0xff;                                   // init = 0xff
  uint8_t data1;

  for (uint8_t n = 0; n < FRAME_LENGTH - 1; n++)
  {
    data1 = data[n];
    for (uint8_t i = 0; i < 8; i++)
    {
      uint8_t tmp = (crc ^ data1) & 0x01;
      crc >>= 1;
      if (tmp) crc ^= 0x8C;
      data1 >>= 1;
    }
  }
  return crc;
}

void W136::DecodeFrame(byte *bytes, struct Frame *frame) {
  frame->IsValid = true;
  frame->ID = 0;
  frame->NewBatteryFlag = false;
  frame->LowBatteryFlag = false;
  frame->ErrorFlag = false;

  frame->HasTemperature = true;
  frame->HasHumidity = true;
  frame->HasRain = true;
  frame->HasWindSpeed = true;
  frame->HasWindDirection = true;
  frame->HasWindGust = true;
  frame->HasPressure = false;
  frame->HasUV = true;
  frame->HasstrikesTotal = true;
  frame->HasstrikesDistance = true;

  frame->Header = bytes[2];
  if (frame->Header == 0x1A) {
    frame->IsValid = true;
  } else {
    frame->IsValid = false;
  }

  frame->CRC = bytes[FRAME_LENGTH - 1];
  if (frame->CRC != CalculateCRC(bytes)) {
    frame->IsValid = false;
  }

  if (frame->IsValid) {
    frame->ID = bytes[0] ;
    frame->NewBatteryFlag = false;
    frame->ErrorFlag = false;
    frame->LowBatteryFlag = bytes[5] & 0x01;                                              // 0=ok, 1=low

    // Temperature (�C)
    frame->Temperature = (float)((int16_t)((bytes[4] << 8) + bytes[3])) / 10.0;
    // Humidity (%rH)
    frame->Humidity = (bytes[1] & 0x0F) + (bytes[1] >> 4) * 10;

    frame->WindDirection = float(bytes[8] & 0x0F) * 22.5;                                   // 0..360°
    frame->WindSpeed = (float)(int16_t)((bytes[10] << 8) + bytes[9]) / 10.0;                      // m/s
    frame->WindGust = (float)(int16_t)((bytes[12] << 8) + bytes[11]) / 10.0;                    // m/s
    frame->Rain = (float)(int16_t)((bytes[14] << 8) + bytes[13]) / 4.0;                    // mm
    frame->UV = (float)bytes[16] / 10.0;
    if (bytes[17] == 0x3F)
      frame->strikesDistance = -1;
    else
      frame->strikesDistance = bytes[17];                                             // km
    frame->strikesTotal = (bytes[20] << 8) + bytes[19];                              // count
  }
}


String W136::AnalyzeFrame(byte *data) {
  struct Frame frame;
  DecodeFrame(data, &frame);

  byte frameLength = W136::FRAME_LENGTH;

  return WSBase::AnalyzeFrame(data, &frame, frameLength, "W136");
}

String W136::GetFhemDataString(byte *data) {
  String fhemString = "";
  if ((data[2]) == 0x1A) {
    struct Frame frame;
    DecodeFrame(data, &frame);
    if (frame.IsValid) {
      //fhemString = BuildFhemDataString(&frame, 6);
      fhemString = BuildKVDataString(&frame, 6);
    }
  }

  return fhemString;
}

bool W136::TryHandleData(byte *data) {
  String fhemString = GetFhemDataString(data);

  if (fhemString.length() > 0) {
    Serial.println(fhemString);
  }

  return fhemString.length() > 0;
}



bool W136::IsValidDataRate(unsigned long dataRate) {
  return dataRate == 4800ul;
}


