#ifndef _TX22IT_h
#define _TX22IT_h

#include "Arduino.h"
#include "SensorBase.h"


class TX22IT : public SensorBase {
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
    
    float Temperature;    // °C 
    byte  Humidity;       // %rH
    float Rain;           // mm
    float WindDirection;  // Degree
    float  WindSpeed;     // m/s
    float  WindGust;      // m/s
  };

  static byte GetFrameLength(byte data[]);
  static byte CalculateCRC(byte data[]);
  static void EncodeFrame(struct TX22IT::Frame *frame, byte bytes[4]);
  static void DecodeFrame(byte *bytes, struct TX22IT::Frame *frame);
  static void AnalyzeFrame(byte *data);
  static bool TryHandleData(byte *data);
  static String GetFhemDataString(struct TX22IT::Frame *frame);


private:
  static float DecodeValue(byte q1, byte q2, byte q3);

};


#endif

