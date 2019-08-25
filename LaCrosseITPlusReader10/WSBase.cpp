#include "WSBase.h"

String WSBase::BuildFhemDataString(struct Frame *frame, byte sensorType) {
  /* Format
  OK WS 60  1   4   193 52    2 88  4   101 15  20   ID=60  21.7°C  52%rH  600mm  Dir.: 112.5°  Wind:15m/s  Gust:20m/s
  OK WS ID  XXX TTT TTT HHH RRR RRR DDD DDD SSS SSS GGG GGG UUU VVV FFF PPP PPP
  |  |  |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |------ Flags *
  |  |  |   |   |   |   |   |   |   |   |   |   |   |   |   |   |-- Light (??? ... ??? lux)                        FF/FF = none
  |  |  |   |   |   |   |   |   |   |   |   |   |   |   |   |------ UV Index (0 ... 13)                            FF/FF = none
  |  |  |   |   |   |   |   |   |   |   |   |   |   |   |---------- WindGust * 10 LSB (0.0 ... 50.0 m/s)           FF/FF = none
  |  |  |   |   |   |   |   |   |   |   |   |   |   |-------------- WindGust * 10 MSB
  |  |  |   |   |   |   |   |   |   |   |   |   |------------------ WindSpeed  * 10 LSB(0.0 ... 50.0 m/s)          FF/FF = none
  |  |  |   |   |   |   |   |   |   |   |   |---------------------- WindSpeed  * 10 MSB
  |  |  |   |   |   |   |   |   |   |   |-------------------------- WindDirection * 10 LSB (0.0 ... 365.0 Degrees) FF/FF = none
  |  |  |   |   |   |   |   |   |   |------------------------------ WindDirection * 10 MSB
  |  |  |   |   |   |   |   |   |---------------------------------- Rain LSB (0 ... 9999 mm)                       FF/FF = none
  |  |  |   |   |   |   |   |-------------------------------------- Rain MSB
  |  |  |   |   |   |   |------------------------------------------ Humidity (1 ... 99 %rH)                        FF = none
  |  |  |   |   |   |---------------------------------------------- Temp * 10 + 1000 LSB (-40 ... +60 �C)          FF/FF = none
  |  |  |   |   |-------------------------------------------------- Temp * 10 + 1000 MSB
  |  |  |   |------------------------------------------------------ Sensor type (1=TX22IT, 2=NodeSensor, 3=WS1080)
  |  |  |---------------------------------------------------------- Sensor ID (1 ... 63)
  |  |------------------------------------------------------------- fix "WS"
  |---------------------------------------------------------------- fix "OK"

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
  if (frame->HasHumidity && (frame->Humidity < 1 || frame->Humidity > 100)) {
    isValid = false;
  }

  if (isValid) {
    pBuf += "OK WS ";
    pBuf += frame->ID;
    pBuf += " ";
    pBuf += sensorType;

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

    if (frame->HasUV && (frame->UV < 0 || frame->UV > 13)) {
      isValid = false;
    }
    // add uvi
    pBuf += AddByte(frame->UV, frame->HasUV);

    // add light
    pBuf += AddWord(frame->Light, frame->HasLight);

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

    // add pressure
    if (frame->HasPressure) {
      pBuf += " ";
      pBuf += (byte)(frame->Pressure >> 8);
      pBuf += " ";
      pBuf += (byte)(frame->Pressure);
    }
  }

  return pBuf;
}


String WSBase::AddWord(word value, bool hasValue) {
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

String WSBase::AddByte(byte value, bool hasValue) {
  String result;
  result += ' ';
  result += hasValue ? value : 0xFF;

  return result;
}

float WSBase::DecodeValue(byte q1, byte q2, byte q3) {
  float result = 0;

  result += q1 * 100;
  result += q2 * 10;
  result += q3;

  return result;
}

String WSBase::AnalyzeFrame(byte *data, Frame *frame, byte frameLength, String prefix) {
  
  String hexinfo;
  String result;
  
  // Show the raw data bytes
  hexinfo += prefix;

  hexinfo += " [";
  for (int i = 0; i < frameLength; i++) {
    hexinfo += String(data[i], HEX);
    if (i < frameLength) {
      hexinfo += " ";
    }
  }
  hexinfo += "]";

  // CRC
  if (!frame->IsValid) {
    hexinfo += " CRC:WRONG";

    if (!m_debug) {
      Serial.println(hexinfo);    
    }

  }
  else {
    hexinfo += " CRC:OK";

    result += prefix;
    
    // Start
    result += " S";
    result += String(frame->Header, HEX);

    // Sensor ID
    result += " ID";
    result += String(frame->ID, HEX);

    // New battery flag
    result += " nBa";
    result += String(frame->NewBatteryFlag, DEC);

    // Low battery flag
    result += " lBa";
    result += String(frame->LowBatteryFlag, DEC);

    // Error flag
    result += " Er";
    result += String(frame->ErrorFlag, DEC);

    // Temperature
    result += " t";
    if (frame->HasTemperature) {
      result += String(frame->Temperature, 1);
    }
    else {
      result += "-";
    }

    // Humidity
    result += " h";
    if (frame->HasHumidity) {
      result += String(frame->Humidity);
    }
    else {
      result += "-";
    }

    // Rain
    result += " r";
    if (frame->HasRain) {
      result += String(frame->Rain, 2);
    }
    else {
      result += "-";
    }

    // Wind speed
    result += " w";
    if (frame->HasWindSpeed) {
      result += String(frame->WindSpeed, 1);      
    }
    else {
      result += "-";
    }

    // Wind direction
    result += " wd";
    if (frame->HasWindDirection) {
      result += String(frame->WindDirection, 0);
    }
    else {
      result += "-";
    }

    // Wind gust
    result += " wg";
    if (frame->HasWindGust) {
      result += String(frame->WindGust, 1);
    }
    else {
      result += "-";
    }

    // UVindex
    result += " uv";
    if (frame->HasUV) {
      result += String(frame->UV, 1);
    }
    else {
      result += "-";
    }

    // Light
    result += " li";
    if (frame->HasLight) {
      result += String(frame->Light, 1);
    }
    else {
      result += "-";
    }

    // CRC
    result += " CRC:";
    result += String(frame->CRC, HEX);

  }

  if (m_debug) {
    Serial.println(hexinfo);    
  }

  return result;
}

String WSBase::BuildKVDataString(struct Frame *frame, byte sensorType) {
  // KeyValue example
  // Format:  KV <Type> <Address> <Key>=<Value>,<Key>=<Value>,<Key>=<Value>, ...
  // Example: KV ADDON 01 Voltage=3.3,UpTime=100
  // -> LGW will send it as KeyValueProtocol to FHEM

  String pBuf = "";
  String pBuf2 = "";
  String pBuf3;
  String sensorTypeName = "";

  // Check if data is in the valid range
  bool isValid = true;
  if (frame->ErrorFlag) {
    isValid = false;
  }

  switch (sensorType) {
    case 3: sensorTypeName = "WH1080"; break;
    case 4: sensorTypeName = "LG"; break;
    case 5: sensorTypeName = "WH25"; break;
    case 6: sensorTypeName = "W136"; break;
    case 7: sensorTypeName = "WH24"; break;
    case 8: sensorTypeName = "FineOffset"; break;
  }


  if (isValid) {
    pBuf += "OK VALUES ";
    pBuf += sensorTypeName;
    pBuf += " ";
    pBuf += frame->ID;
    pBuf += " ";

  //  pBuf += "Header=";
  //  pBuf += frame->Header;
  //  pBuf += ",";

    // add temperature
    if (frame->HasTemperature && (frame->Temperature < -40.0 || frame->Temperature > 59.9)) {
      isValid = false;
    } else {
      pBuf += "Temperature=";
      pBuf += frame->Temperature;
      pBuf += ",";
    }
    // add humidity
    if (frame->HasHumidity && (frame->Humidity < 1 || frame->Humidity > 100)) {
      isValid = false;
    } else {
      pBuf += "Humidity=";
      pBuf += frame->Humidity;
      pBuf += ",";
    }
    // add pressure
    if (frame->HasPressure) {
      if ( (frame->Pressure > 500 || frame->Pressure < 2000)) {
        pBuf += "Pressure=";
        pBuf += frame->Pressure;
        pBuf += ",";
      }
    }
    // add rain
    if (frame->HasRain) {
      pBuf += "Rain=";
      pBuf += frame->Rain;
      pBuf += ",";
    }
    // add wind speed
    if (frame->HasWindSpeed) {
      pBuf += "WindSpeed=";
      pBuf += frame->WindSpeed;
      pBuf += ",";
    }
    // add wind direction
    if (frame->HasWindDirection) {
      pBuf += "WindDirection=";
      pBuf += frame->WindDirection;
      pBuf += ",";
    }

    // add gust
    if (frame->HasWindGust) {
      pBuf += "WindGust=";
      pBuf += frame->WindGust;
      pBuf += ",";
    }
    // add UV
    if (frame->HasUV) {
      pBuf += "UV=";
      pBuf += frame->UV;
      pBuf += ",";
    }
    // add strikesDistance
    if (frame->HasstrikesDistance) {
      pBuf += "strikesDistance=";
      pBuf += frame->strikesDistance;
      pBuf += ",";
    }
    // add strikesTotal
    if (frame->HasstrikesTotal) {
      pBuf += "strikesTotal=";
      pBuf += frame->strikesTotal;
      pBuf += ",";
    }
    // add Flags
    if (frame->NewBatteryFlag) {
      pBuf += "NewBatteryFlag=";
      pBuf += frame->NewBatteryFlag;
      pBuf += ",";
    }

    if (frame->ErrorFlag) {
      pBuf += "ErrorFlag=";
      pBuf += frame->ErrorFlag;
      pBuf += ",";
    }
    if (frame->LowBatteryFlag) {
      pBuf += "LowBatteryFlag=";
      pBuf += frame->LowBatteryFlag;
      pBuf += ",";
    }

  }
   return pBuf;
}

