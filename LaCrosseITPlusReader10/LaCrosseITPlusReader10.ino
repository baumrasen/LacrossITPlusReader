// Tested with JeeLink v3 (2012-02-11)
// polling RFM12B to decode FSK iT+ with a JeeNode/JeeLink from Jeelabs.
// Supported devices: see FHEM wiki
// info    : http://forum.jeelabs.net/node/110
//           http://fredboboss.free.fr/tx29/tx29_sw.php
//           http://www.f6fbb.org/domo/sensors/
//           http://www.mikrocontroller.net/topic/67273 
//           benedikt.k org rinie,marf,joop 1 nov 2011, slightly modified by Rufik (r.markiewicz@gmail.com)
// Changelog: 2012-02-11: initial release 1.0
//            2014-03-14: I have this in SubVersion, so no need to do it here

#define PROGNAME         "LaCrosseITPlusReader"
#define PROGVERS         "10.1p" 

#include "SPI.h"
#include "RFMxx.h"
#include "SensorBase.h"
#include "LaCrosse.h"
#include "LevelSenderLib.h"
#include "EMT7110.h"
#include "WT440XH.h"
#include "TX38IT.h"
#include "TX22IT.h"
#include "JeeLink.h"
#include "Help.h"
#include "BMP180.h"
#include <Wire.h>
#include "InternalSensors.h"
#include "CustomSensor.h"

// --- Configuration ---------------------------------------------------------------------------------------------------
#define RECEIVER_ENABLED       1                     // Set to 0 if you don't want to receive 
#define USE_OLD_IDS            0                     // Set to 1 to use the old ID calcualtion

// The following settings can also be set from FHEM
#define ENABLE_ACTIVITY_LED    1         // <n>a     set to 0 if the blue LED bothers
unsigned long DATA_RATE_S1   = 17241ul;  // <n>c     use one of the possible data rates (for transmit on RFM #1)
bool DEBUG                   = 0;        // <n>d     set to 1 to see debug messages
unsigned long INITIAL_FREQ   = 868300;   // <n>f     initial frequency in kHz (5 kHz steps, 860480 ... 879515) 
int ALTITUDE_ABOVE_SEA_LEVEL = 0;        // <n>h     altituide above sea level
byte TOGGLE_MODE_R1          = 3;        // <n>m     bits 1: 17.241 kbps, 2 : 9.579 kbps, 4 : 8.842 kbps (for RFM #1)
byte TOGGLE_MODE_R2          = 3;        // <n>M     bits 1: 17.241 kbps, 2 : 9.579 kbps, 4 : 8.842 kbps (for RFM #2)
                                         // <n>o     set HF-parameter e.g. 50305o for RFM12 or 1,4o for RFM69
byte PASS_PAYLOAD            = 0;        // <n>p     transmitted the payload on the serial port 1: all, 2: only undecoded data
unsigned long DATA_RATE_R1   = 17241ul;  // <n>r     use one of the possible data rates (for RFM #1)
unsigned long DATA_RATE_R2   = 9579ul;   // <n>R     use one of the possible data rates (for RFM #2)
                                         // <id,..>s send the bytes to the address id
uint16_t TOGGLE_INTERVAL_R1  = 0;        // <n>t     0=no toggle, else interval in seconds (for RFM #1)
uint16_t TOGGLE_INTERVAL_R2  = 0;        // <n>T     0=no toggle, else interval in seconds (for RFM #2)
                                         // v        show version
                                         // x        test command 
bool RELAY                   = 0;        // <n>y     if 1 all received packets will be retransmitted  
bool ANALYZE_FRAMES          = 0;        // <n>z     set to 1 to display analyzed frame data instead of the normal data


// --- Variables -------------------------------------------------------------------------------------------------------
unsigned long lastToggleR1 = 0;
unsigned long lastToggleR2 = 0;
byte commandData[32];
byte commandDataPointer = 0;
RFMxx rfm1(11, 12, 13, 10, 2, true);
RFMxx rfm2(11, 12, 13, 8, 3, false);


JeeLink jeeLink;
InternalSensors internalSensors;

static unsigned long ConvertDataRate(unsigned long value) {
 unsigned long result = 0;
  switch (value) {
    case 0:
      result = 17241ul;
      break;
    case 1:
      result = 9579ul;
      break;
    case 2:
      result = 8842ul;
      break;
    default:
      result = value;
      break;
  }
  return result;
}

static void HandleSerialPort(char c) {
  static unsigned long value;
  unsigned long dataRate = 0;

  if (c == ',') {
    commandData[commandDataPointer++] = value;
    value = 0;
  }
  else if ('0' <= c && c <= '9') {
    value = 10 * value + c - '0';
  }
  else if (('a' <= c && c <= 'z') || ('A' <= c && c <= 'Z')) {
    switch (c) {
    case 'd':
      // DEBUG
      SetDebugMode(value);
      break;
    case 'h':
      // height
      internalSensors.SetAltitudeAboveSeaLevel(value);
      break;
    case 'x':
      // Tests
      HandleCommandX(value);
      break;
    case 'a':
      // Activity LED    
      jeeLink.EnableLED(value);
      break;
    case 'r':
    case 'R':
      // Data rate
      dataRate = ConvertDataRate(value);
      if (c == 'r') {
        DATA_RATE_R1 = dataRate;
        rfm1.SetDataRate(DATA_RATE_R1);
      }
      else {
        if (rfm2.IsConnected()) {
          DATA_RATE_R2 = dataRate;
          rfm2.SetDataRate(DATA_RATE_R2);
        }
      }
      break;
    case 'c':
      // TX Data rate
      dataRate = ConvertDataRate(value);
      DATA_RATE_S1 = dataRate;
      break;
    case 'm':
      TOGGLE_MODE_R1 = value;
      break;
    case 'M':
      TOGGLE_MODE_R2 = value;
      break;
    case 'p':
      PASS_PAYLOAD = value;
      break;
    case 't':
      // Toggle data rate
      TOGGLE_INTERVAL_R1 = value;
      break;
    case 'T':
      // Toggle data rate
      TOGGLE_INTERVAL_R2 = value;
      break;
    case 'v':
      // Version info
      HandleCommandV();
      break;

    case 's':
      // Send
      commandData[commandDataPointer] = value;
      HandleCommandS(commandData, ++commandDataPointer);
      commandDataPointer = 0;
      break;

    case 'o':
    case 'O':
      // Set HF parameter
      commandData[commandDataPointer] = value;
      HandleCommandO(c == 'O' ? 2 : 1, value, commandData, ++commandDataPointer);
      commandDataPointer = 0;
      break;

    case 'f':
      rfm1.SetFrequency(value);
      break;

    case 'F':
      if (rfm2.IsConnected()) {
        rfm2.SetFrequency(value);
      }
      break;

    case 'y':
      RELAY = value;
      break;

    case 'z':
      ANALYZE_FRAMES = value;
      break;

    default:
      HandleCommandV();
      #ifndef NOHELP
      Help::Show();
      #endif
      break;
    }
    value = 0;
  }
  else if (' ' < c && c < 'A') {
    HandleCommandV();
    #ifndef NOHELP
    Help::Show();
    #endif
  }
}

void SetDebugMode(boolean mode) {
  DEBUG = mode;
  LevelSenderLib::SetDebugMode(mode);
  WT440XH::SetDebugMode(mode);
  rfm1.SetDebugMode(mode);
  rfm2.SetDebugMode(mode);

}

void HandleCommandO(byte rfmNbr, unsigned long value, byte *data, byte size) {
  // 50305o (is 0xC481) for RFM12 or 1,4o for RFM69
  if (size == 1 && rfm1.GetRadioType() == RFMxx::RFM12B) {
    if (rfmNbr == 1) {
      rfm1.SetHFParameter(value);
    }
    else if (rfmNbr == 2) {
      rfm2.SetHFParameter(value);
    }
  }
  else if (size == 2 && rfm1.GetRadioType() == RFMxx::RFM69CW) {
    if (rfmNbr == 1) {
      rfm1.SetHFParameter(data[0], data[1]);
    }
    else if (rfmNbr == 2) {
      rfm2.SetHFParameter(data[0], data[1]);
    }
  }


}

void HandleCommandS(byte *data, byte size) {
  rfm1.EnableReceiver(false);

  struct CustomSensor::Frame frame;
  frame.ID = data[0];
  frame.NbrOfDataBytes = size -1;

  for (int i = 0; i < frame.NbrOfDataBytes; i++) {
    frame.Data[i] = data[i+1];
  }

  CustomSensor::SendFrame(&frame, &rfm1, DATA_RATE_S1);


  rfm1.EnableReceiver(true);
}


// This function is for testing 
void HandleCommandX(byte value) {

}

void HandleCommandV() {
  Serial.print("\n[");
  Serial.print(PROGNAME);
  Serial.print('.');
  Serial.print(PROGVERS);

  Serial.print(" (");
  Serial.print(rfm1.GetRadioName());

  Serial.print(" f:");
  Serial.print(rfm1.GetFrequency());
  
  if (TOGGLE_INTERVAL_R1) {
    Serial.print(" t:");
    Serial.print(TOGGLE_INTERVAL_R1);
    Serial.print("~");
    Serial.print(TOGGLE_MODE_R1);
  }
  else {
    Serial.print(" r:");
    Serial.print(rfm1.GetDataRate());
    
  }
  Serial.print(")");

  if(rfm2.IsConnected()) {
    Serial.print(" + (");
    Serial.print(rfm2.GetRadioName());
    Serial.print(" f:");
    Serial.print(rfm2.GetFrequency());
    if (TOGGLE_INTERVAL_R2) {
      Serial.print(" t:");
      Serial.print(TOGGLE_INTERVAL_R2);
      Serial.print("~");
      Serial.print(TOGGLE_MODE_R2);
    }
    else {
      Serial.print(" r:");
      Serial.print(rfm2.GetDataRate());

    }
    Serial.print(")");
  }

  if (internalSensors.HasBMP180()) {
    Serial.print(" + BMP180");
  }

  Serial.println(']');
}

void HandleReceivedData(RFMxx *rfm) {
  rfm->EnableReceiver(false);

  byte payload[PAYLOADSIZE];
  rfm->GetPayload(payload);

  if (ANALYZE_FRAMES) {
    TX22IT::AnalyzeFrame(payload);
    LaCrosse::AnalyzeFrame(payload);
    LevelSenderLib::AnalyzeFrame(payload);
    EMT7110::AnalyzeFrame(payload);
    TX38IT::AnalyzeFrame(payload);
    CustomSensor::AnalyzeFrame(payload);
    Serial.println();
  }
  else if (PASS_PAYLOAD == 1) {
    jeeLink.Blink(1);
    for (int i = 0; i < PAYLOADSIZE; i++) {
      Serial.print(payload[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  else {
    jeeLink.Blink(1);

    if (DEBUG) {
      Serial.print("\nEnd receiving, HEX raw data: ");
      for (int i = 0; i < 16; i++) {
        Serial.print(payload[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }

    byte frameLength = 0;

    // Try TX22IT (WS 1600)
    if (TX22IT::IsValidDataRate(rfm->GetDataRate()) && TX22IT::TryHandleData(payload)) {
      frameLength = TX22IT::GetFrameLength(payload);
    }

    // Try LaCrosse like TX29DTH
    else if (LaCrosse::IsValidDataRate(rfm->GetDataRate()) && LaCrosse::TryHandleData(payload)) {
      frameLength = LaCrosse::FRAME_LENGTH;
    }

    // Try LevelSender
    else if (LevelSenderLib::IsValidDataRate(rfm->GetDataRate()) && LevelSenderLib::TryHandleData(payload)) {
      frameLength = LevelSenderLib::FRAME_LENGTH;
    }

    // Try EMT7110
    else if (EMT7110::IsValidDataRate(rfm->GetDataRate()) && EMT7110::TryHandleData(payload)) {
      frameLength = EMT7110::FRAME_LENGTH;
    }

    // Try WT440XH
    else if (WT440XH::IsValidDataRate(rfm->GetDataRate()) && WT440XH::TryHandleData(payload)) {
      frameLength = WT440XH::FRAME_LENGTH;
    }

    // Try TX38IT
    else if (TX38IT::IsValidDataRate(rfm->GetDataRate()) && TX38IT::TryHandleData(payload)) {
      frameLength = TX38IT::FRAME_LENGTH;
    }
    // Try CustomSensor
    else if (CustomSensor::IsValidDataRate(rfm->GetDataRate()) && CustomSensor::TryHandleData(payload)) {
      frameLength = CustomSensor::GetFrameLength(payload);
    }
    else if (PASS_PAYLOAD == 2) {
      for (int i = 0; i < PAYLOADSIZE; i++) {
        Serial.print(payload[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }


    if (RELAY && frameLength > 0) {
      delay(64);
      rfm->SendArray(payload, frameLength);
      if (DEBUG) { Serial.println("Relayed"); }
    }

  }
  rfm->EnableReceiver(true);
}

void HandleDataRateToggle(RFMxx *rfm, unsigned long *lastToggle, unsigned long *dataRate, uint16_t interval, byte toggleMode) {
  if (interval > 0) {
    // After about 50 days millis() will overflow to zero 
    if (millis() < *lastToggle) {
      *lastToggle = 0;
    }

    if (millis() > *lastToggle + interval * 1000) {
      // Bits 1: 17.241 kbps, 2 : 9.579 kbps, 4 : 8.842 kbps

      if (*dataRate == 8842ul) {
        if (toggleMode & 2) {
          *dataRate = 9579ul;
        }
        else if (toggleMode & 1) {
          *dataRate = 17241ul;
        }
      }
      else if (*dataRate == 9579ul) {
        if (toggleMode & 1) {
          *dataRate = 17241ul;
        }
        else if (toggleMode & 4) {
          *dataRate = 8842ul;
        }
      }
      else if (*dataRate == 17241ul) {
        if (toggleMode & 4) {
          *dataRate = 8842ul;
        }
        else if (toggleMode & 2) {
          *dataRate = 9579ul;
        }
      }

      rfm->SetDataRate(*dataRate);
      *lastToggle = millis();

    }
  }
}

// **********************************************************************
void loop(void) {
  // Handle the commands from the serial port
  // ----------------------------------------
  if (Serial.available()) {
    HandleSerialPort(Serial.read());
  }

  // Periodically send own sensor data
  // ---------------------------------
  internalSensors.TryHandleData();

  // Handle the data reception
  // -------------------------
  if (RECEIVER_ENABLED) {
    rfm1.Receive();
    if (rfm1.PayloadIsReady()) {
      HandleReceivedData(&rfm1);
    }
    
    if(rfm2.IsConnected()) {
      rfm2.Receive();
      if (rfm2.PayloadIsReady()) {
        HandleReceivedData(&rfm2);
      }
    }
    
  }
 
  // Handle the data rate
  // --------------------
  HandleDataRateToggle(&rfm1, &lastToggleR1, &DATA_RATE_R1, TOGGLE_INTERVAL_R1, TOGGLE_MODE_R1);
  HandleDataRateToggle(&rfm2, &lastToggleR2, &DATA_RATE_R2, TOGGLE_INTERVAL_R2, TOGGLE_MODE_R2);

}


void setup(void) {
  Serial.begin(57600);
  delay(200);
  if (DEBUG) {
    Serial.println("*** LaCrosse weather station wireless receiver for IT+ sensors ***");
  }

  SetDebugMode(DEBUG);
  LaCrosse::USE_OLD_ID_CALCULATION = USE_OLD_IDS;
  
  internalSensors.TryInitializeBMP180();
  internalSensors.SetAltitudeAboveSeaLevel(ALTITUDE_ABOVE_SEA_LEVEL);

  jeeLink.EnableLED(ENABLE_ACTIVITY_LED);
  lastToggleR1 = millis();
  
  rfm1.InitialzeLaCrosse();
  rfm1.SetFrequency(INITIAL_FREQ);
  rfm1.SetDataRate(DATA_RATE_R1);
  rfm1.EnableReceiver(true);
  
  if(rfm2.IsConnected()) {
    rfm2.InitialzeLaCrosse();
    rfm2.SetFrequency(INITIAL_FREQ);
    rfm2.SetDataRate(DATA_RATE_R2);
    rfm2.EnableReceiver(true);
  }
  
  
  if (DEBUG) {
    Serial.println("Radio setup complete. Starting to receive messages");
  }

  // FHEM needs this information
  delay(1000);
  HandleCommandV();

}





