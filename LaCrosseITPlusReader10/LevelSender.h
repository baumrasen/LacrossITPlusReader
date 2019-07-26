#ifndef _LEVELSENDER_h
#define _LEVELSENDER_h

#include "Arduino.h"
#include "SensorBase.h"

class LevelSender : public SensorBase {
public:
  struct Frame {
    byte  Header;
    byte  ID;
    float Level;
    float Temperature;
    float Voltage;
    byte  CRC;
    bool  IsValid;
  };
  static byte CalculateCRC(byte *data);
  static const byte FRAME_LENGTH = 6;
  static void EncodeFrame(struct Frame *frame, byte *bytes);
  static void DecodeFrame(byte *data, struct Frame *frame);
  static void AnalyzeFrame(byte *data);
  static bool TryHandleData(byte *data);
  static String GetFhemDataString(struct LevelSender::Frame *frame);


};

#endif

