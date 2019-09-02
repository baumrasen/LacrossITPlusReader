#ifndef _WSBASE_h
#define _WSBASE_h

#include "Arduino.h"
#include "SensorBase.h"

class WSBase : public SensorBase {
public:
  struct Frame {
    byte  Header;
    byte  ID;
    bool  NewBatteryFlag;
    bool  LowBatteryFlag;
    bool  ErrorFlag;
    byte  CRC;
    bool  IsValid;

    bool  HasTemperature;
    bool  HasHumidity;
    bool  HasRain;
    bool  HasWindSpeed;
    bool  HasWindDirection;
    bool  HasWindGust;
    bool  HasPressure;
    bool  HasUV;
    bool  HasLight;

    float Temperature;    // °C 
    byte  Humidity;       // %rH
    float Rain;           // mm
    float WindDirection;  // Degree
    float WindSpeed;      // m/s
    float WindGust;       // m/s
    int Pressure;         // hPa
    int UV;
    byte  Light_b1;
    byte  Light_b2;
    byte  Light_b3;
  };

protected:
  static String BuildFhemDataString(struct Frame *frame, byte sensorType);
  static String AddWord(word value, bool hasValue);
  static String AddByte(byte value, bool hasValue);
  static float DecodeValue(byte q1, byte q2, byte q3);
  static String AnalyzeFrame(byte *data, Frame *frame, byte frameLength, String prefix);
};

#endif

