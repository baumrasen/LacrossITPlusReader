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
#define PROGVERS         "10.1d"

#include "RFM12.h"
#include "SensorBase.h"
#include "LaCrosse.h"
#include "LevelSenderLib.h"
#include "EMT7110.h"
#include "WT440XH.h"
#include "JeeLink.h"
#include "Transmitter.h"
#include "Help.h"

// --- Configuration ---------------------------------------------------------
#define RECEIVER_ENABLED      1                     // Set to 0 if you don't want to receive 
#define ANALYZE_FRAMES        0                     // Set to 1 to display analyzed frame data instead of the normal data
#define RELAY                 0                     // If 1 all received packets will be retransmitted  
#define ENABLE_ACTIVITY_LED   1                     // set to 0 if the blue LED bothers
#define USE_OLD_IDS           0                     // Set to 1 to use the old ID calcualtion
bool    DEBUG               = 0;                    // set to 1 to see debug messages
RFM12::DataRates DATA_RATE  = RFM12::DataRate17241; // use one of the possible data rates
uint16_t TOGGLE_DATA_RATE   = 0;                    // 0=no toggle, else interval in seconds


// --- Variables --------------------------------------------------------------
unsigned long lastToggle = 0;
byte commandData[32];
byte commandDataPointer = 0;
byte receiveBuffer[64];
byte receiveBufferPointer = 0;
RFM12 rfm(11, 12, 13, 10, 2);
JeeLink jeeLink;
Transmitter transmitter(&rfm);


static void HandleSerialPort(char c) {
  static byte value;

  if (c == ',') {
    commandData[commandDataPointer++] = value;
    value = 0;
  }
  else if ('0' <= c && c <= '9') {
    value = 10 * value + c - '0';
  }
  else if ('a' <= c && c <= 'z') {
    switch (c) {
    case 'd':     
      // DEBUG
      SetDebugMode(value);
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
      // Data rate
      DATA_RATE = value ? RFM12::DataRate9579 : RFM12::DataRate17241;
      rfm.SetDataRate(DATA_RATE);
      break;
    case 't':     
      // Toggle data rate
      TOGGLE_DATA_RATE = value;
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

    case 'i':
      commandData[commandDataPointer] = value;
      HandleCommandI(commandData, ++commandDataPointer);
      commandDataPointer = 0;
      break;

    case 'c':
      commandData[commandDataPointer] = value;
      HandleCommandC(commandData, ++commandDataPointer);
      commandDataPointer = 0;
      break;

    default:
      HandleCommandV();
      Help::Show();
      break;
    }
    value = 0;
  }
  else if (' ' < c && c < 'A') {
    HandleCommandV();
    Help::Show();
  }
}

void SetDebugMode(boolean mode) {
  DEBUG = mode;
  LevelSenderLib::SetDebugMode(mode);
  WT440XH::SetDebugMode(mode);
  rfm.SetDebugMode(mode);
}

void HandleCommandS(byte *data, byte size) {
  if (size == 4){
    rfm.EnableReceiver(false);

    // Calculate the CRC
    data[LaCrosse::FRAME_LENGTH - 1] = LaCrosse::CalculateCRC(data);

    rfm.SendArray(data, LaCrosse::FRAME_LENGTH);

    rfm.EnableReceiver(true);
  }
}

void HandleCommandI(byte *values, byte size){
  // 14,43,20,0i  -> ID 14, Interval 4.3 Seconds, reset NewBatteryFlagafter 20 minutes, 17.241 kbps
  if (size == 4){
    transmitter.SetParameters(values[0],
                              values[1] * 100,
                              true,
                              values[2] * 60000 + millis(),
                              values[3] == 0 ? RFM12::DataRate17241 : RFM12::DataRate9579);
    transmitter.Enable(true);
  }
  else if (size == 1 && values[0] == 0){
    transmitter.Enable(false);
  }

  
}

void HandleCommandC(byte *values, byte size){
  // 2,1,9,44c    -> Temperatur  21,9°C and 44% humidity
  // 129,4,5,77c  -> Temperatur -14,5°C and 77% humidity
  // To set a negative temperature set bit 7 in the first byte (add 128)
  if (size == 4){
    float temperature = (values[0] & 0b0111111) * 10 + values[1] + values[2] * 0.1;
    if (values[0] & 0b10000000) {
      temperature *= -1;
    }
    
    transmitter.SetValues(temperature, values[3]);
  }
}


// This function is for testing 
void HandleCommandX(byte value) {
  LaCrosse::Frame frame;
  frame.ID = 20;
  frame.NewBatteryFlag = true;
  frame.Bit12 = false;
  frame.Temperature = value;
  frame.WeakBatteryFlag = false;
  frame.Humidity = value;

  if (DEBUG) {
    Serial.print("TX: T=");
    Serial.print(frame.Temperature);
    Serial.print(" H=");
    Serial.print(frame.Humidity);
    Serial.print(" NB=");
    Serial.print(frame.NewBatteryFlag);
    Serial.println();
  }

  byte bytes[LaCrosse::FRAME_LENGTH];
  LaCrosse::EncodeFrame(&frame, bytes);
  rfm.SendArray(bytes, LaCrosse::FRAME_LENGTH);

  rfm.EnableReceiver(RECEIVER_ENABLED);
}

void HandleCommandV() {
  Serial.print("\n[");
  Serial.print(PROGNAME);
  Serial.print('.');
  Serial.print(PROGVERS);
  Serial.print(" @");
  if (TOGGLE_DATA_RATE) {
    Serial.print("AutoToggle ");
    Serial.print(TOGGLE_DATA_RATE);
    Serial.print(" Seconds");
  }
  else {
    Serial.print(DATA_RATE == RFM12::DataRate9579 ? "9.579 kbps" : "17.241 kbps");
  }
  Serial.println(']');
}

// **********************************************************************
void loop(void) {
  static unsigned long lastReceiveTime = 0;

  // Handle the commands from the serial port
  // ----------------------------------------
  if (Serial.available()) {
    HandleSerialPort(Serial.read());
  }

  // Handle the data rate
  // --------------------
  if (TOGGLE_DATA_RATE > 0) {
    // After about 50 days millis() will overflow to zero 
    if (millis() < lastToggle) {
      lastToggle = 0;
    }

    if (millis() > lastToggle + TOGGLE_DATA_RATE * 1000) {
      if (DATA_RATE == RFM12::DataRate9579) {
        DATA_RATE = RFM12::DataRate17241;
      }
      else {
        DATA_RATE = RFM12::DataRate9579;
      }

      rfm.SetDataRate(DATA_RATE);
      lastToggle = millis();

    }
  }

  // Priodically transmit
  // --------------------
  if (transmitter.Transmit()) {
    jeeLink.Blink(2);
    rfm.EnableReceiver(RECEIVER_ENABLED);
    receiveBufferPointer = 0;
  }

  // Handle the data reception
  // -------------------------
  if (RECEIVER_ENABLED) {
    if (rfm.FifoHasData()) {
      receiveBuffer[receiveBufferPointer++] = rfm.GetByteFromFifo();
      lastReceiveTime = millis();
    }

    if ((receiveBufferPointer > 0 && millis() > lastReceiveTime + 50) || receiveBufferPointer >= 32) {
      rfm.EnableReceiver(false);
      
      if(ANALYZE_FRAMES) {
        LaCrosse::AnalyzeFrame(receiveBuffer);
        LevelSenderLib::AnalyzeFrame(receiveBuffer);
        EMT7110::AnalyzeFrame(receiveBuffer);
        Serial.println();
      }
      else {
        jeeLink.Blink(1);

        if (DEBUG) {
          Serial.print("\nEnd receiving, HEX raw data: ");
          for (int i = 0; i < receiveBufferPointer -1; i++) {
            Serial.print(receiveBuffer[i], HEX);
            Serial.print(" ");
          }
          Serial.println();
        }

        
        byte frameLength = 0;

        // Try LaCrosse like TX29DTH
        if (LaCrosse::TryHandleData(receiveBuffer)) {
          frameLength = LaCrosse::FRAME_LENGTH;
        }

        // Try LevelSender
        else if (LevelSenderLib::TryHandleData(receiveBuffer)) {
          frameLength = LevelSenderLib::FRAME_LENGTH;
        }

        // Try EMT7110
        else if (EMT7110::TryHandleData(receiveBuffer)) {
          frameLength = EMT7110::FRAME_LENGTH;
        }

        // Try WT440XH
        else if (WT440XH::TryHandleData(receiveBuffer)) {
          frameLength = WT440XH::FRAME_LENGTH;
        }
       

        if (RELAY && frameLength > 0) {
          delay(64);
          rfm.SendArray(receiveBuffer, frameLength);
          if (DEBUG) { Serial.println("Relayed"); }
        }



      }

      receiveBufferPointer = 0;
      rfm.EnableReceiver(true);
    }
  }
 
}


void setup(void) {
  Serial.begin(57600);
  delay(200);
  if (DEBUG) {
    Serial.println("*** LaCrosse weather station wireless receiver for IT+ sensors ***");
  }

  SetDebugMode(DEBUG);
  LaCrosse::USE_OLD_ID_CALCULATION = USE_OLD_IDS;


  jeeLink.EnableLED(ENABLE_ACTIVITY_LED);
  lastToggle = millis();
  
  rfm.InitialzeLaCrosse();
  rfm.SetDataRate(DATA_RATE);
  transmitter.Enable(false);
  rfm.EnableReceiver(true);

  if (DEBUG) {
    Serial.println("Radio setup complete. Starting to receive messages");
  }

  // FHEM needs this information
  HandleCommandV();

}





