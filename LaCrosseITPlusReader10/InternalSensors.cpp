#include "InternalSensors.h"

InternalSensors::InternalSensors() {
  m_hasBMP180 = false;
  m_lastMeasurement = 0;
}

bool InternalSensors::TryInitializeBMP180() {
  m_bmp.SetAltitudeAboveSeaLevel(0);
  m_hasBMP180 = m_bmp.TryInitialize();

  return m_hasBMP180;
}

bool InternalSensors::HasBMP180() {
  return m_hasBMP180;
}

void InternalSensors::SetAltitudeAboveSeaLevel(int altitude) {
  m_bmp.SetAltitudeAboveSeaLevel(altitude);
}


String InternalSensors::BuildFhemDataString(struct Frame *frame) {
  String result = "";

  // Check if data is in the valid range
  bool isValid = true;
  
  if (frame->Temperature < -40.0 || frame->Temperature > 85.0) {
    isValid = false;
  }
  
  
  if(isValid) {
    result += "OK WS ";
    result += frame->ID;
    result += " 2";

    // add temperature
    int temp = frame->Temperature * 10 + 1000;
    result += " ";
    result += (byte)(temp >> 8);
    result += " ";
    result += (byte)(temp);

    // no humidity
    result += " 255";

    // no rain
    result += " 255 255";

    // no wind direction
    result += " 255 255";

    // no wind speed
    result += " 255 255";

    // no wind gust
    result += " 255 255";

    // add Flags
    byte flags = 0;
    if (frame->NewBatteryFlag) {
      flags += 1;
    }
    if (frame->LowBatteryFlag) {
      flags += 4;
    }
    result += " ";
    result += flags;

    // add pressure
    result += " ";
    result += (byte)(frame->Pressure >> 8);
    result += " ";
    result += (byte)(frame->Pressure);

  }
  
  return result;
}

 String InternalSensors::GetFhemDataString(){
  String fhemString = "";

  struct Frame frame;

  if (m_hasBMP180) {
    if (millis() >= m_lastMeasurement + 10000) {
      m_lastMeasurement = millis();

      frame.ID = 0;
      frame.LowBatteryFlag = false;
      frame.NewBatteryFlag = false;
      frame.IsValid = true;
      frame.Temperature = m_bmp.GetTemperature();
      frame.Pressure = m_bmp.GetPressure();

      if (frame.IsValid) {
        fhemString = BuildFhemDataString(&frame);
      }

    }

  }

  return fhemString;
}

 bool InternalSensors::TryHandleData(){
   String fhemString = GetFhemDataString();

   if (fhemString.length() > 0) {
     Serial.println(fhemString);
   }

   return fhemString.length() > 0;
 }

