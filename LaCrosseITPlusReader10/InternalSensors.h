#ifndef _INTERNALSENSORS_h
#define _INTERNALSENSORS_h

#include "TX22IT.h"
#include "BMP180.h"


class InternalSensors {
public: 
  struct Frame {
    byte  ID;
    bool  NewBatteryFlag;
    bool  LowBatteryFlag;
    bool  IsValid;

    bool  HasTemperature;
    bool  HasPressure;

    float Temperature;    // °C 
    int  Pressure;        // hPa
  };

  InternalSensors();
  bool TryInitializeBMP180();
  bool HasBMP180();
  bool TryHandleData();
  String GetFhemDataString();
  void SetAltitudeAboveSeaLevel(int altitude);

protected:
  bool m_hasBMP180;
  BMP180 m_bmp;
  unsigned long m_lastMeasurement;
  String BuildFhemDataString(struct InternalSensors::Frame *frame);
};


#endif

