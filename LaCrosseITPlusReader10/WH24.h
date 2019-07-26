#ifndef _WH24_h
#define _WH24_h

#include "Arduino.h"
#include "WSBase.h"
#include "SensorBase.h"
#include "LaCrosse.h"



class WH24 : public WSBase {
    float  Pressure;        // hPa
  public:
    static const byte FRAME_LENGTH = 17;
    static byte CalculateCRC(byte data[]);
    static void DecodeFrame(byte *bytes, struct WH24::Frame *frame);
    static String AnalyzeFrame(byte *data);
    static bool TryHandleData(byte *data);
    static String GetFhemDataString(byte *data);
    static bool IsValidDataRate(unsigned long dataRate);

  protected:
    //static String BuildFhemDataString(struct WH24::Frame *frame, byte sensorType);
    //static String BuildKVDataString(struct WH24::Frame *frame, byte sensorType);
 uint8_t crc8(uint8_t const message[], unsigned nBytes, uint8_t polynomial, uint8_t init);

};


#endif

