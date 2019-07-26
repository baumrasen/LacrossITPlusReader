#include "TX22IT.h"

/*
TX22-IT  8842 kbps  868.3 MHz
-----------------------------
Message Format:
SSSS.DDDD DDAE.LQQQ TTTT.VVVV VVVV.VVVV ... CCCC.CCCC 
Data - organized in nibbles - are structured as follows (example with blanks added for clarity):
a 5a 5 0 628 1 033 2 000 3 e00 4 000 bd

data always starts with "a"
from next 1.5 nibbles (here 5a) the 6 msb are identifier of transmitter,
bit 1 indicates acquisition/synchronizing phase
bit 0 will be 1 in case of error
next bit is the low battery flag
next three bits (here 5) is count of quartets transmitted
up to 5 quartets of data follow
each quartet starts with a type indicator (here 0,1,2,3,4)
0: temperature: 3 nibbles bcd coded tenth of °C plus 400 (here 628-400 = 22.8°C)
1: humidity: 3 nibbles bcd coded (here 33 %rH)
2: rain: 3 nibbles, counter of contact closures
3: wind: first nibble direction of wind vane (multiply by 22.5 to obtain degrees, here 0xe*22.5 = 315 degrees)
   next two nibbles wind speed in m/s
4: gust: speed in m/s
next two bytes (here bd) are crc.
During acquisition/synchronizing phase (abt. 5 hours) all 5 quartets are sent, see example above. Thereafter
data strings contain only a few ( 1 up to 3) quartets, so data strings are not always of equal length.


                    |--- acquisition/synchronizing phase
                    ||-- Error
        "A"  -Addr.-|| Nbr.Q
        SSSS.DDDD DDAE.LQQQ  T          H          R          W          G           CRC
TX22IT [A    1    D    3                1  0 7 2   2  0 1 B   3  C F E               C4    ] CRC:OK S:A ID:7 NewBatt:0 Error:1 Temp:---   Hum:72  Rain:27.00 Wind:25.40m/s from:270.00 Gust:---      CRC:C4
TX22IT [A    1    D    2                           2  0 1 B   3  D F E               3A    ] CRC:OK S:A ID:7 NewBatt:0 Error:1 Temp:---   Hum:--- Rain:27.00 Wind:25.40m/s from:292.50 Gust:---      CRC:3A
TX22IT [A    1    D    2                           2  0 1 B   3  E F E               17    ] CRC:OK S:A ID:7 NewBatt:0 Error:1 Temp:---   Hum:--- Rain:27.00 Wind:25.40m/s from:315.00 Gust:---      CRC:17
TX22IT [A    1    C    3                1  0 7 3   2  0 1 B              4  0  0  0  8A    ] CRC:OK S:A ID:7 NewBatt:0 Error:0 Temp:---   Hum:73  Rain:27.00 Wind:---      from:---    Gust:0.00 m/s CRC:8A
TX22IT [A    1    C    1                           2  0 1 B                          E     ] CRC:OK S:A ID:7 NewBatt:0 Error:0 Temp:---   Hum:--- Rain:27.00 Wind:---      from:---    Gust:---      CRC:E
TX22IT [A    1    C    2     0  5 5 3              2  0 1 B                          19    ] CRC:OK S:A ID:7 NewBatt:0 Error:0 Temp:15.30 Hum:--- Rain:27.00 Wind:---      from:---    Gust:---      CRC:19

*/

byte TX22IT::CalculateCRC(byte data[]) {
  byte CRC[8];
  byte bits[8];
  int i,j;
  byte val;
  byte DoInvert;
  byte result;
  
  byte len = TX22IT::GetFrameLength(data) -1;

  for(i=0; i<8; i++) {
    CRC[i] = 0;
  }

  for(j=0; j<len; j++) {
    val = data[j];
    
    for(i=0; i<8; i++) {
      switch(i) {
        case 0: if((val & 0x80) != 0) { bits[i] = 1; } else { bits[i] = 0; } break;
        case 1: if((val & 0x40) != 0) { bits[i] = 1; } else { bits[i] = 0; } break;
        case 2: if((val & 0x20) != 0) { bits[i] = 1; } else { bits[i] = 0; } break;
        case 3: if((val & 0x10) != 0) { bits[i] = 1; } else { bits[i] = 0; } break;
        case 4: if((val & 0x8) != 0) { bits[i] = 1; } else { bits[i] = 0; } break;
        case 5: if((val & 0x4) != 0) { bits[i] = 1; } else { bits[i] = 0; } break;
        case 6: if((val & 0x2) != 0) { bits[i] = 1; } else { bits[i] = 0; } break;
        case 7: if((val & 0x1) != 0) { bits[i] = 1; } else { bits[i] = 0; } break;
      }
  
      if(bits[i] == 1) {
        DoInvert = 1 ^ CRC[7];
      }
      else {
        DoInvert = 0 ^ CRC[7];
      }  
      
      CRC[7] = CRC[6];
      CRC[6] = CRC[5];
      CRC[5] = CRC[4] ^ DoInvert;
      CRC[4] = CRC[3] ^ DoInvert;
      CRC[3] = CRC[2];
      CRC[2] = CRC[1];
      CRC[1] = CRC[0];
      CRC[0] = DoInvert;
    }
  }

  result = (CRC[7] << 7) |
           (CRC[6] << 6) |
           (CRC[5] << 5) |
           (CRC[4] << 4) |
           (CRC[3] << 3) |
           (CRC[2] << 2) |
           (CRC[1] << 1) |
           (CRC[0]);
              
  return result;
}

void TX22IT::DecodeFrame(byte *bytes, struct Frame *frame) {
  frame->IsValid = true;
  frame->Header = 0;
  frame->ID = 0;
  frame->NewBatteryFlag = false;
  frame->LowBatteryFlag = false;
  frame->ErrorFlag = false; 

  frame->HasTemperature = false;
  frame->HasHumidity = false;
  frame->HasRain = false;
  frame->HasWindSpeed = false;
  frame->HasWindDirection = false;
  frame->HasWindGust = false;
  
  frame->Temperature = 0;
  frame->Humidity = 0;
  frame->Rain = 0;
  frame->WindDirection = 0;
  frame->WindSpeed = 0;
  frame->WindGust = 0;
  frame->CRC = 0;

  frame->CRC = bytes[GetFrameLength(bytes) -1];
  if (frame->CRC != CalculateCRC(bytes)) {
    frame->IsValid = false;
  }

  frame->Header = (bytes[0] & 0xF0) >> 4;
  if (frame->Header != 0xA) {
    frame->IsValid = false;
  }

  if (frame->IsValid) {
    frame->ID = ((bytes[0] & 0xF) << 2) | ((bytes[1] & 0xC0) >> 6);

    frame->NewBatteryFlag = (bytes[1] & 0b00100000) > 0;
    frame->ErrorFlag = (bytes[1] & 0b00010000) > 0;
    frame->LowBatteryFlag = (bytes[1] & 0xF) >> 3;

    byte ct = bytes[1] & 0x7;
    for (int i = 0; i < ct; i++) {
      byte byte1 = bytes[2 + i * 2];
      byte byte2 = bytes[3 + i * 2];

      byte type = (byte1 & 0xF0) >> 4;
      byte q1 = (byte1 & 0xF);
      byte q2 = (byte2 & 0xF0) >> 4;
      byte q3 = (byte2 & 0xF);

      switch (type) {
        case 0:
          frame->HasTemperature = true;
          frame->Temperature = (DecodeValue(q1, q2, q3) - 400) / 10.0;
          if (frame->Temperature > 60 || frame->Temperature < -40) {
            frame->IsValid = false;
          }
          break;
        
        case 1:
          frame->HasHumidity= true;
          frame->Humidity = DecodeValue(q1, q2, q3);
          if (frame->Humidity > 100 || frame->Humidity < 0) {
            frame->IsValid = false;
          }
          break;

        case 2:
          frame->HasRain = true;
          frame->Rain = q1 * 256 + q2 * 16 + q3;
          break;

        case 3:
          frame->HasWindDirection = true;
          frame->HasWindSpeed = true;

          frame->WindDirection = q1 * 22.5;
          frame->WindSpeed = (q2 * 16 + q3) / 10.0;
          break;

        case 4:
          frame->HasWindGust = true;
          frame->WindGust = (q2 * 16 + q3) / 10.0;
          break;
      }

    }
    
  }
}

String AddWord(word value, bool hasValue) {
  String result;

  if (!hasValue) {
    value = 0xFFFF;
  }

  result += ' ';
  result += (byte)(value >> 8);
  result += ' ';
  result += (byte)(value);

  return result;
}

String AddByte(byte value, bool hasValue) {
  String result;
  result += ' ';
  result += hasValue ? value : 0xFF;
  
  return result;
}


String TX22IT::BuildFhemDataString(struct Frame *frame) {
  /* Format
   OK WS 60  1   4   193 52    2 88  4   101 15  20   ID=60  21.7°C  52%rH  600mm  Dir.: 112.5°  Wind:15m/s  Gust:20m/s 
   OK WS ID  XXX TTT TTT HHH RRR RRR DDD DDD SSS SSS GGG GGG FFF
   |  |  |   |   |   |   |   |   |   |   |   |   |   |   |   |-- Flags *
   |  |  |   |   |   |   |   |   |   |   |   |   |   |   |------ WindGust * 10 LSB (0.0 ... 50.0 m/s)           FF/FF = none 
   |  |  |   |   |   |   |   |   |   |   |   |   |   |---------- WindGust * 10 MSB
   |  |  |   |   |   |   |   |   |   |   |   |   |-------------- WindSpeed  * 10 LSB(0.0 ... 50.0 m/s)          FF/FF = none  
   |  |  |   |   |   |   |   |   |   |   |   |------------------ WindSpeed  * 10 MSB
   |  |  |   |   |   |   |   |   |   |   |---------------------- WindDirection * 10 LSB (0.0 ... 365.0 Degrees) FF/FF = none
   |  |  |   |   |   |   |   |   |   |-------------------------- WindDirection * 10 MSB
   |  |  |   |   |   |   |   |   |------------------------------ Rain LSB (0 ... 9999 mm)                       FF/FF = none
   |  |  |   |   |   |   |   |---------------------------------- Rain MSB
   |  |  |   |   |   |   |-------------------------------------- Humidity (1 ... 99 %rH)                        FF = none
   |  |  |   |   |   |------------------------------------------ Temp * 10 + 1000 LSB (-40 ... +60 °C)          FF/FF = none
   |  |  |   |   |---------------------------------------------- Temp * 10 + 1000 MSB
   |  |  |   |-------------------------------------------------- Sensor type (1=TX22)
   |  |  |------------------------------------------------------ Sensor ID (1 ... 63)
   |  |--------------------------------------------------------- fix "WS"
   |------------------------------------------------------------ fix "OK"

   * Flags: 128  64  32  16  8   4   2   1        
                                 |   |   |
                                 |   |   |-- New battery
                                 |   |------ ERROR
                                 |---------- Low battery
*/
  
  String pBuf = "";

  // Check if data is in the valid range
  bool isValid = true;
  if (frame->ErrorFlag) {
    isValid = false;
  }
  if (frame->HasTemperature && (frame->Temperature < -40.0 || frame->Temperature > 59.9)) {
    isValid = false;
  }
  if(frame->HasHumidity && (frame->Humidity < 1 || frame->Humidity > 99)) {
    isValid = false;
  }
  
  if(isValid) {
    pBuf += "OK WS ";
    pBuf += frame->ID;
    pBuf += " 1";

    // add temperature
    pBuf += AddWord(frame->Temperature * 10 + 1000, frame->HasTemperature);

    // add humidity
    pBuf += AddByte(frame->Humidity, frame->HasHumidity);

    // add rain
    pBuf += AddWord(frame->Rain, frame->HasRain);

    // add wind direction
    pBuf += AddWord(frame->WindDirection * 10, frame->HasWindDirection);

    // add wind speed
    pBuf += AddWord(frame->WindSpeed * 10, frame->HasWindSpeed);

    // add gust
    pBuf += AddWord(frame->WindGust * 10, frame->HasWindGust);


    // add Flags
    byte flags = 0;
    if (frame->NewBatteryFlag) {
      flags += 1;
    }
    if (frame->ErrorFlag) {
      flags += 2;
    }
    if (frame->LowBatteryFlag) {
      flags += 4;
    }
    pBuf += AddByte(flags, true);
  }
  
  return pBuf;
}


byte TX22IT::GetFrameLength(byte data[]) {
  return 3 + 2 * (data[1] & 0x7);
}


float TX22IT::DecodeValue(byte q1, byte q2, byte q3) {
  float result = 0;

  result += q1 * 100;
  result += q2 * 10;
  result += q3;

  return result;
}

void TX22IT::AnalyzeFrame(byte *data) {
  bool onlyValidData = true;

  struct Frame frame;
  DecodeFrame(data, &frame);

  byte frameLength = TX22IT::GetFrameLength(data);

  if (frame.IsValid || !onlyValidData) {
    // Show the raw data bytes
    Serial.print("TX22IT [");
    for (int i = 0; i < frameLength; i++) {
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.print("]");
  }

  // CRC
  if (!frame.IsValid) {
    if (!onlyValidData) {
      Serial.println(" CRC:WRONG");
    }
  }
  else {
    Serial.print(" CRC:OK");

    // Start
    Serial.print(" S:");
    Serial.print(frame.Header, HEX);

    // Sensor ID
    Serial.print(" ID:");
    Serial.print(frame.ID, HEX);

    // New battery flag
    Serial.print(" NewBatt:");
    Serial.print(frame.NewBatteryFlag, DEC);
    
    // Low battery flag
    Serial.print(" LowBatt:");
    Serial.print(frame.LowBatteryFlag, DEC);

    // Error flag
    Serial.print(" Error:");
    Serial.print(frame.ErrorFlag, DEC);

    // Temperature
    Serial.print(" Temp:");
    if (frame.HasTemperature) {
      Serial.print(frame.Temperature);
    }
    else {
      Serial.print("---");
    }

    // Humidity
    Serial.print(" Hum:");
    if (frame.HasHumidity) {
      Serial.print(frame.Humidity);
    }
    else {
      Serial.print("---");
    }

    // Rain
    Serial.print(" Rain:");
    if (frame.HasRain) {
      Serial.print(frame.Rain);
    }
    else {
      Serial.print("---");
    }

    // Wind speed
    Serial.print(" Wind:");
    if (frame.HasWindSpeed) {
      Serial.print(frame.WindSpeed);
      Serial.print("m/s");
    }
    else {
      Serial.print("---");
    }

    // Wind direction
    Serial.print(" from:");
    if (frame.HasWindDirection) {
      Serial.print(frame.WindDirection);
    }
    else {
      Serial.print("---");
    }

    // Wind gust
    Serial.print(" Gust:");
    if (frame.HasWindGust) {
      Serial.print(frame.WindGust);
      Serial.print(" m/s");
    }
    else {
      Serial.print("---");
    }

    // CRC
    Serial.print(" CRC:");
    Serial.print(frame.CRC, HEX);

    Serial.println();
  }

}

String TX22IT::GetFhemDataString(byte *data) {
  String fhemString = "";

  if ((data[0] & 0xA0) == 0xA0) {
    struct Frame frame;
    DecodeFrame(data, &frame);
    if (frame.IsValid) {
      fhemString = BuildFhemDataString(&frame);
    }
  }

  return fhemString;
}

bool TX22IT::TryHandleData(byte *data) {
  String fhemString = GetFhemDataString(data);

  if (fhemString.length() > 0) {
    Serial.println(fhemString);
  }
 
  return fhemString.length() > 0;
}



void TX22IT::EncodeFrame(struct Frame *frame, byte bytes[4]) {

}

bool TX22IT::IsValidDataRate(unsigned long dataRate) {
  return dataRate == 8842ul;
}