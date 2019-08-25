#include "WH24.h"


byte WH24::CalculateCRC(byte message[]) {          

unsigned nBytes = 15 ;
uint8_t polynomial = 0x31;
uint8_t init = 0x00;

uint8_t remainder = init;
unsigned byte, bit;

    for (byte = 0; byte < nBytes; ++byte) {
        remainder ^= message[byte];
        for (bit = 0; bit < 8; ++bit) {
            if (remainder & 0x80) {
                remainder = (remainder << 1) ^ polynomial;
            } else {
                remainder = (remainder << 1);
            }
        }
    }
    return remainder;
}

/* Fine Offset Electronics WH24, WH65B, HP1000 and derivatives Temperature/Humidity/Pressure sensor protocol
 *
 * The sensor sends a package each ~16 s with a width of ~11 ms. The bits are PCM modulated with Frequency Shift Keying
 *
 * Example:
 *      [00] {196} d5 55 55 55 55 16 ea 12 5f 85 71 03 27 04 01 00 25 00 00 80 00 00 47 83 9
 *   aligned {199} 1aa aa aa aa aa 2d d4 24 bf 0a e2 06 4e 08 02 00 4a 00 01 00 00 00 8f 07 2
 *                                       24 4F 9B 61 F2 5B 0  0  7  2F 0  0  0  0  0  23
 * Payload:                              FF II DD VT TT HH WW GG RR RR UU UU LL LL LL CC BB
 * Reading: id: 191, temp: 11.8 C, humidity: 78 %, wind_dir 266 deg, wind_speed: 1.12 m/s, gust_speed 2.24 m/s, rainfall: 22.2 mm
 *
 * The WH65B sends the same data with a slightly longer preamble and postamble
 *         {209} 55 55 55 55 55 51 6e a1 22 83 3f 14 3a 08 00 00 00 08 00 10 00 00 04 60 a1 00 8
 * aligned  {208} a aa aa aa aa aa 2d d4 24 50 67 e2 87 41 00 00 00 01 00 02 00 00 00 8c 14 20 1
 * Payload:                              FF II DD VT TT HH WW GG RR RR UU UU LL LL LL CC BB
 *
 * Preamble:  aa aa aa aa aa
 * Sync word: 2d d4
 * Payload:   FF II DD VT TT HH WW GG RR RR UU UU LL LL LL CC BB
 *
 * FF = Family Code, fixed 0x24
 * II = Sensor ID, set on battery change
 * DD = Wind direction
 * V = Various bits, D11S, wind dir 8th bit, wind speed 8th bit
 * T TT = Temperature (+40*10), top bit is low battery flag
 * HH = Humidity
 * WW = Wind speed
 * GG = Gust speed
 * RR RR = rainfall counter
 * UU UU = UV value
 * LL LL LL = light value
 * CC = CRC checksum of the 15 data bytes
 * BB = Bitsum (sum without carry, XOR) of the 16 data bytes
 */
#define MODEL_WH24 24 /* internal identifier for model WH24, family code is always 0x24 */
#define MODEL_WH65B 65 /* internal identifier for model WH65B, family code is always 0x24 */

void WH24::DecodeFrame(byte *bytes, struct Frame *frame) {

  uint8_t tempkorr = 0;
  frame->IsValid = true;
  frame->ID = 0;
  frame->NewBatteryFlag = false;
  frame->LowBatteryFlag = true;
  frame->ErrorFlag = false;

  frame->HasTemperature = true;
  frame->HasHumidity = true;
  frame->HasRain = true;
  frame->HasWindSpeed = true;
  frame->HasWindDirection = true;
  frame->HasWindGust = true;
  frame->HasPressure = false;
  frame->HasUV = true;
  frame->HasLight = true;
  frame->HasstrikesTotal = false;
  frame->HasstrikesDistance = false;


  frame->Header = bytes[0] ;
  if (frame->Header == 0x24) {      // Check for family code 0x24
    frame->IsValid = true;
  } else {
    frame->IsValid = false;
  }

  if (frame->IsValid) {
      // Verify checksum, same as other FO Stations: Reverse 1Wire CRC (poly 0x131)
    uint8_t crc = WH24::CalculateCRC(bytes);
    uint8_t checksum = 0;

    for (size_t n = 0; n < 16; ++n) {
        checksum += bytes[n];
    }
    
 //   if (crc != bytes[15] || checksum != bytes[16]) {
    if (crc != bytes[15]) {
        if (m_debug) {
          Serial.print("## CRC FAIL ### - WH24: ");
          Serial.println(crc, HEX);
        }
        frame->IsValid = false;
    } else {
        frame->IsValid = true;
     }
  }

  if (frame->IsValid) {
    
    frame->ID = (bytes[1]);

    // int low_battery     = (bytes[3] & 0x08) >> 3;
    int low_battery     = (bytes[3] & 0x08) >> 3;
    frame->LowBatteryFlag = low_battery;

            if (m_debug) {
          Serial.print("WH24 - id: ");
          Serial.print(frame->ID);

          Serial.print("   batt: ");
          Serial.print(frame->LowBatteryFlag);
        }
        
    // Temperature (°C)
    int temp = ((bytes[3] & 0x07) << 8) | bytes[4]; // 0x7ff if invalid
    frame->Temperature = ((temp * 0.1) - 40.0);     // range -40.0-60.0 C
    // Humidity (%rH)
    frame->Humidity = bytes[5];                     // 0xff if invalid
   // frame->Pressure = ((bytes[4] << 8) | bytes[5]) / 10.0;
   
        if (m_debug) {
          Serial.print("   t: ");
          Serial.print(frame->Temperature);

          Serial.print("   h: ");
          Serial.print(frame->Humidity);
        }

    // Wind speed (m/s)
    int wind_speed_raw  = bytes[6] | (bytes[3] & 0x10) << 4; // 0x1ff if invalid
     float wind_speed_factor, rain_cup_count;
    // Wind speed factor is 1.12 m/s (1.19 per specs?) for WH24, 0.51 m/s for WH65B
    // Rain cup each count is 0.3mm for WH24, 0.01inch (0.254mm) for WH65B
    //if (model == MODEL_WH24) { // WH24
    //    wind_speed_factor = 1.12;
    //    rain_cup_count = 0.3;
    //} else { // WH65B
        wind_speed_factor = 0.51;
        rain_cup_count = 0.254;
    //}
    // Wind speed is scaled by 8, wind speed = raw / 8 * 1.12 m/s (0.51 for WH65B)
    frame->WindSpeed = wind_speed_raw * 0.125 * wind_speed_factor;
    
    // Wind gust (m/s)
     int gust_speed_raw  = bytes[7];             // 0xff if invalid
    // Wind gust is unscaled, multiply by wind speed factor 1.12 m/s
    frame->WindGust = gust_speed_raw * wind_speed_factor;
    
            if (m_debug) {
          Serial.print("   ws: ");
          Serial.print(frame->WindSpeed);

          Serial.print("   wg: ");
          Serial.print(frame->WindGust);
        }

    //  Rain 
    int rainfall_raw    = bytes[8] << 8 | bytes[9]; // rain tip counter
    frame->Rain = rainfall_raw * rain_cup_count; // each tip is 0.3mm / 0.254mm
    
    // Wind direction (degree  N=0, NNE=22.5, S=180, ... )
    frame->WindDirection = bytes[2] | (bytes[3] & 0x80) << 1; // range 0-359 deg, 0x1ff if invalid
    
        if (m_debug) {
          Serial.print("   r: ");
          Serial.print(frame->Rain);

          Serial.print("   wd: ");
          Serial.print(frame->WindDirection);
        }

    int uv_raw          = bytes[10] << 8 | bytes[11];               // range 0-20000, 0xffff if invalid
    int light_raw       = bytes[12] << 16 | bytes[13] << 8 | bytes[14]; // 0xffffff if invalid
    float light_lux     = light_raw * 0.1; // range 0.0-300000.0lux
    // Light = value/10 ; Watts/m Sqr. = Light/683 ;  Lux to W/m2 = Lux/126

    // UV value   UVI
    // 0-432      0
    // 433-851    1
    // 852-1210   2
    // 1211-1570  3
    // 1571-2017  4
    // 2018-2450  5
    // 2451-2761  6
    // 2762-3100  7
    // 3101-3512  8
    // 3513-3918  9
    // 3919-4277  10
    // 4278-4650  11
    // 4651-5029  12
    // >=5230     13
    int uvi_upper[] = {432, 851, 1210, 1570, 2017, 2450, 2761, 3100, 3512, 3918, 4277, 4650, 5029};
    int uv_index   = 0;
    while (uv_index < 13 && uvi_upper[uv_index] < uv_raw) ++uv_index; 
    frame->UV =  uv_index;
    frame->Light =  light_lux;

            if (m_debug) {

          Serial.print("   uv_raw: ");
          Serial.print(uv_raw,1);

          Serial.print("   uv: ");
          Serial.print(frame->UV);

          Serial.print("   light: ");
          Serial.println(light_lux);
        }

  }
}


String WH24::AnalyzeFrame(byte *data) {
  struct Frame frame;
  DecodeFrame(data, &frame);

  byte frameLength = WH24::FRAME_LENGTH;

  return WSBase::AnalyzeFrame(data, &frame, frameLength, "FineOffset");
}

String WH24::GetFhemDataString(byte *data) {
  String fhemString2 = "";
  struct Frame frame2;
  DecodeFrame(data, &frame2);
  if (frame2.IsValid) {
    fhemString2 = BuildFhemDataString(&frame2, 8);
    // fhemString2 = BuildKVDataString(&frame2, 7);
  }
  return fhemString2;
}

bool WH24::TryHandleData(byte *data) {
  String fhemString = GetFhemDataString(data);

  if (fhemString.length() > 0) {
    Serial.println(fhemString);
  }

  return fhemString.length() > 0;
}



bool WH24::IsValidDataRate(unsigned long dataRate) {
  return dataRate == 17241ul;
}
