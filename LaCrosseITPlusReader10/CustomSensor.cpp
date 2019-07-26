#include "CustomSensor.h"

// Message-Format
// --------------
//  SSSSSSSS  IIIIIIII  BBBBBBBB  DDDDDDDD ... DDDDDDDD  CCCCCCCC
//  |      |  |      |  |      |  |                   |  |       |
//  |      |  |      |  |      |  |                   |  `---------- CRC
//  |      |  |      |  |      |  |                   |
//  |      |  |      |  |      |  `---------------------- Data
//  |      |  |      |  |      | 
//  |      |  |      |  `--------- Nbr of data bytes
//  |      |  |      |
//  |      |  `--------- ID (00 ... FF)
//  |      | 
//  `--------- START (CC)
void CustomSensor::DecodeFrame(byte *bytes, struct CustomSensor::Frame *frame) {
  frame->IsValid = true;
  frame->ID = 0;
  frame->CRC = 0;

  byte len = GetFrameLength(bytes);
  frame->CRC = bytes[len - 1];
  if (frame->CRC != CalculateCRC(bytes, len -1)) {
    frame->IsValid = false;
  }

  if (bytes[0] != CUSTOM_SENSOR_HEADER) {
    frame->IsValid = false;
  }

  if (frame->IsValid) {
    frame->ID = bytes[1];
    frame->NbrOfDataBytes = bytes[2];

    for (int i = 3; i < len -1; i++) {
      frame->Data[i-3] = bytes[i];
    }
    
  }
}

// ----------------------------------------------------------------
byte CustomSensor::GetFrameLength(byte data[]) {
  return 4 + data[2];
}

// ----------------------------------------------------------------
String CustomSensor::BuildFhemDataString(struct CustomSensor::Frame *frame) {
  String pBuf = "";
  
  /* Format
  OK  CC  11  1   2   3   4   5   ...
  OK  CC  ID  B01 B02 B02 B04 B05 ...
  |   |   |   |   |   |   |   |   
  |   |   |   |   |   |   |   |---------------------------------- Byte 5
  |   |   |   |   |   |   |-------------------------------------- Byte 4
  |   |   |   |   |   |------------------------------------------ Byte 3
  |   |   |   |   |---------------------------------------------- Byte 2
  |   |   |   |-------------------------------------------------- Byte 1
  |   |   |------------------------------------------------------ ID
  |   |---------------------------------------------------------- fix "CC"
  |-------------------------------------------------------------- fix "OK"
  */
  
  pBuf += "OK CC ";
  pBuf += frame->ID;
  pBuf += " ";

  for (int i = 0; i < frame->NbrOfDataBytes; i++) {
    pBuf += frame->Data[i];
    pBuf += " ";
  }
  
  return pBuf;
}

// ----------------------------------------------------------------
void CustomSensor::SendFrame(struct CustomSensor::Frame *frame, RFMxx rfm) {
  byte payload[CS_PL_BUFFER_SIZE];
  EncodeFrame(frame, payload);

  rfm.EnableReceiver(false);
  unsigned long currentDataRate = rfm.GetDataRate();
  rfm.SetDataRate(17241);
  rfm.SendArray(payload, CustomSensor::GetFrameLength(payload));
  rfm.SetDataRate(currentDataRate);
  rfm.EnableReceiver(true);
}

// ----------------------------------------------------------------
void CustomSensor::AnalyzeFrame(byte *data) {
  struct CustomSensor::Frame frame;
  DecodeFrame(data, &frame);

  byte frameLength = CustomSensor::GetFrameLength(data);

  // Show the raw data bytes
  Serial.print("CustomSensor [");
  for (int i = 0; i < frameLength; i++) {
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print("] ");

  // CRC
  if (!frame.IsValid) {
    Serial.println(" CRC:WRONG");
  }
  else {
    Serial.print(" CRC:OK");

    // Sensor ID
    Serial.print(" ID:0x");
    Serial.print(frame.ID, HEX);

    // Size
    Serial.print(" NbrOfDataBytes:");
    Serial.print(frame.NbrOfDataBytes, DEC);
    
    // Data
    Serial.print(" Data:");
    for (int i = 0; i < frame.NbrOfDataBytes; i++) {
      Serial.print("0x");
      Serial.print(frame.Data[i], HEX);
      Serial.print(" ");
    }

    // CRC
    Serial.print(" CRC:0x");
    Serial.print(frame.CRC, HEX);

    Serial.println();
  }
}

// ----------------------------------------------------------------
String CustomSensor::GetFhemDataString(byte *data) {
  String fhemString = "";

  if (data[0] == CUSTOM_SENSOR_HEADER) {
    struct Frame frame;
    DecodeFrame(data, &frame);
    if (frame.IsValid) {
      fhemString = BuildFhemDataString(&frame);
    }
  }

  return fhemString;
}

// ----------------------------------------------------------------
bool CustomSensor::TryHandleData(byte *data) {
  String fhemString = GetFhemDataString(data);

  if (fhemString.length() > 0) {
    Serial.println(fhemString);
  }
 
  return fhemString.length() > 0;
}

// ----------------------------------------------------------------
void CustomSensor::EncodeFrame(struct Frame *frame, byte bytes[CS_PL_BUFFER_SIZE]) {
  
  for (int i=0; i < CS_PL_BUFFER_SIZE; i++) { bytes[i] = 0; }
  
  // Start and ID
  bytes[0] = CUSTOM_SENSOR_HEADER;
  bytes[1] = frame->ID;
  bytes[2] = frame->NbrOfDataBytes;
  
  // Data
  for (int i=0; i < frame->NbrOfDataBytes; i++) {
    bytes[i+3] = frame->Data[i];
  }

  // CRC
  bytes[3 + frame->NbrOfDataBytes] = CalculateCRC(bytes, 3 + frame->NbrOfDataBytes);
  
}

// ----------------------------------------------------------------
bool CustomSensor::IsValidDataRate(unsigned long dataRate) {
  return dataRate == 17241ul;
}