#include "WSBase.h"

String WSBase::BuildFhemDataString(struct Frame *frame, byte sensorType) {
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
  |  |  |   |-------------------------------------------------- Sensor type (1=TX22IT, 2=NodeSensor, 3=WS1080)
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
  if (frame->HasHumidity && (frame->Humidity < 1 || frame->Humidity > 99)) {
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

void WSBase::AnalyzeFrame(byte *data, Frame *frame, byte frameLength, String prefix) {
  // Show the raw data bytes
  Serial.print(prefix);
  Serial.print(" [");
  for (int i = 0; i < frameLength; i++) {
    Serial.print(data[i], HEX);
    if (i < frameLength) {
      Serial.print(" ");
    }
  }
  Serial.print("]");

  // CRC
  if (!frame->IsValid) {
    Serial.println(" CRC:WRONG");
  }
  else {
    Serial.print(" CRC:OK");

    // Start
    Serial.print(" S:");
    Serial.print(frame->Header, HEX);

    // Sensor ID
    Serial.print(" ID:");
    Serial.print(frame->ID, HEX);

    // New battery flag
    Serial.print(" NewBatt:");
    Serial.print(frame->NewBatteryFlag, DEC);

    // Low battery flag
    Serial.print(" LowBatt:");
    Serial.print(frame->LowBatteryFlag, DEC);

    // Error flag
    Serial.print(" Error:");
    Serial.print(frame->ErrorFlag, DEC);

    // Temperature
    Serial.print(" Temp:");
    if (frame->HasTemperature) {
      Serial.print(frame->Temperature);
    }
    else {
      Serial.print("---");
    }

    // Humidity
    Serial.print(" Hum:");
    if (frame->HasHumidity) {
      Serial.print(frame->Humidity);
    }
    else {
      Serial.print("---");
    }

    // Rain
    Serial.print(" Rain:");
    if (frame->HasRain) {
      Serial.print(frame->Rain);
    }
    else {
      Serial.print("---");
    }

    // Wind speed
    Serial.print(" Wind:");
    if (frame->HasWindSpeed) {
      Serial.print(frame->WindSpeed);
      Serial.print("m/s");
    }
    else {
      Serial.print("---");
    }

    // Wind direction
    Serial.print(" from:");
    if (frame->HasWindDirection) {
      Serial.print(frame->WindDirection);
    }
    else {
      Serial.print("---");
    }

    // Wind gust
    Serial.print(" Gust:");
    if (frame->HasWindGust) {
      Serial.print(frame->WindGust);
      Serial.print(" m/s");
    }
    else {
      Serial.print("---");
    }

    // CRC
    Serial.print(" CRC:");
    Serial.print(frame->CRC, HEX);

    Serial.println();
  }

}