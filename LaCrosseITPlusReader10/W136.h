#ifndef _W136_h
#define _W136_h

#include "Arduino.h"
#include "WSBase.h"
#include "SensorBase.h"
#include "LaCrosse.h"
#include "CustomSensor.h"



class W136 : public WSBase {
  public:
    static const byte PAYLOAD_SIZE = 22;
    static const byte FRAME_LENGTH = 22;
    static byte CalculateCRC(byte data[]);
    static void DecodeFrame(byte *bytes, struct W136::Frame *frame);
    static String AnalyzeFrame(byte *data);
    static bool TryHandleData(byte *data);
    static String GetFhemDataString(byte *data);
    static bool IsValidDataRate(unsigned long dataRate);
   // static String BuildFhemDataString(struct W137::Frame *frame, byte sensorType);
   //  static String BuildKVDataString(struct W137::Frame *frame, byte sensorType);

  protected:
   


};


#endif

