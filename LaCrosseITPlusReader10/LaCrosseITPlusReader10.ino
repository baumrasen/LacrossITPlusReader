// Tested with JeeLink v3 (2012-02-11)
// polling RFM12B to decode FSK iT+ with a JeeNode/JeeLink from Jeelabs.
// Supported devices: see the u command
// info    : http://forum.jeelabs.net/node/110
//           http://fredboboss.free.fr/tx29/tx29_sw.php
//           http://www.f6fbb.org/domo/sensors/
//           http://www.mikrocontroller.net/topic/67273 
//           benedikt.k org rinie,marf,joop 1 nov 2011, slightly modified by Rufik (r.markiewicz@gmail.com)
// Changelog: 2012-02-11: initial release 1.0
//            2014-03-14: I have this in SubVersion, so no need to type it here
#define PROGNAME         "LaCrosseITPlusReader"
#define PROGVERS         "10.0f"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

// --- possible data rates --------------------------------------------------- 
#define DATA_RATE_17  0xC613        // 17.241 kbps for TX29DTH-IT
#define DATA_RATE_9   0xC623        //  9.579 kbps for TX35DTH-IT

// --- Configuration ---------------------------------------------------------
bool DEBUG               = 0;             // set to 1 to see debug messages
bool ENABLE_ACTIVITY_LED = 1;             // set to 0 if the blue LED bothers
int  DATA_RATE           = DATA_RATE_17;  // use one of the data rates defined above
int  TOGGLE_DATA_RATE    = 0;             // 0=no toggle, else interval in seconds
bool RECEIVER_ENABLED    = 1;             // Set to 0 if you don't want to receive 
bool ANALYZE_FRAMES      = 0;             // Set to 1 to display analyzed frame data instead of the normal data




// --- Other settings ---------------------------------------------------------
#define CRC_POLY              0x31  // CRC-8 = 0x31 is for: x8 + x5 + x4 + 1
#define NO_HUMINITY_AVAILABLE 106
#define FRAME_LENGHT          5

// --- Structures -------------------------------------------------------------
struct lacross_message {
  byte  header;
  byte  id;
  bool  newBatteryFlag;
  bool  bit12;
  float temperature;
  bool  weakBatteryFlag;
  byte  humidity;
  byte  crc;
  bool  isValid;
};

struct tx_data {
  bool enabled;
  byte dataRate;
  byte id;
  word interval;
  unsigned long newBatteryFlagResetTime;
  bool newBatteryFlag;
  float temperature;
  byte humidity;
  unsigned long lastTransmit;
};

// --- Variables --------------------------------------------------------------
unsigned long lastToggle = 0;
byte commandData[32];
byte commandDataPointer = 0;
byte receiveBuffer[FRAME_LENGHT];
byte receiveBufferPointer = FRAME_LENGHT;
tx_data transmitData;

/**
*	http://depa.usst.edu.cn/chenjq/www2/SDesign/JavaScript/CRCcalculation.htm
*
*	CRC order  (1..64): 8
*	CRC polynom  (hex): 31
*
*	data in hex: 9EC5576A -> crc BD
*/

// http://stackoverflow.com/questions/12015112/checksum-crc-calculation
// <summary>A variant of CRC-8</summary>
// <param name="data">Pass here the first 4 bytes of data, e.g. { 0x4E 0x50 0x92 0x33 }</param>
// <returns>The computed SRC value, e.g. 0xA1 for the data specified above.</returns>

// 0x31 is for: x8 + x5 + x4 + 1
#define CRC_POLY 0x31
uint8_t crc8(uint8_t *data, int len) {
  int i, j;
  uint8_t res = 0;
  for (j = 0; j<len; j++) {
    uint8_t val = data[j];
    for (i = 0; i < 8; i++) {
      uint8_t tmp = (uint8_t)((res ^ val) & 0x80);
      res <<= 1;
      if (0 != tmp) {
        res ^= CRC_POLY;
      }
      val <<= 1;
    }
  }
  return res;
}

float calculate_temperature(uint8_t *bcd) {
  float t = 0;
  t += bcd[0] * 100.0;
  t += bcd[1] * 10.0;
  t += bcd[2] * 1.0;
  t = t / 10;
  t -= 40;
  return t; 
}

/*
* Message Format:
*
* .- [0] -. .- [1] -. .- [2] -. .- [3] -. .- [4] -.
* |       | |       | |       | |       | |       |
* SSSS.DDDD DDN_.TTTT TTTT.TTTT WHHH.HHHH CCCC.CCCC
* |  | |     ||  |  | |  | |  | ||      | |       |
* |  | |     ||  |  | |  | |  | ||      | `--------- CRC
* |  | |     ||  |  | |  | |  | |`-------- Humidity
* |  | |     ||  |  | |  | |  | |
* |  | |     ||  |  | |  | |  | `---- weak battery
* |  | |     ||  |  | |  | |  |
* |  | |     ||  |  | |  | `----- Temperature T * 0.1
* |  | |     ||  |  | |  |
* |  | |     ||  |  | `---------- Temperature T * 1
* |  | |     ||  |  |
* |  | |     ||  `--------------- Temperature T * 10
* |  | |     | `--- new battery
* |  | `---------- ID
* `---- START
*
*/


void encodeSensorFrame(struct lacross_message msg, byte *frame) {
  for (int i = 0; i < 5; i++) { frame[i] = 0; }
  
  // ID
  frame[0] = 9 << 4;
  frame[0] |= msg.id >> 2;
  frame[1] = (msg.id & 0b00000011) << 6;

  // NewBatteryFlag
  frame[1] |= msg.newBatteryFlag << 5;

  // Bit12
  frame[1] |= msg.bit12 << 4;

  // Temperature
  float temp = msg.temperature + 40.0;
  frame[1] |= (int)(temp / 10);
  frame[2] |= ((int)temp % 10) << 4;
  frame[2] |= (int)(fmod(temp, 1) * 10 + 0.5);

  // Humidity
  frame[3] = msg.humidity;

  // WeakBatteryFlag
  frame[3] |= msg.weakBatteryFlag << 7;

  // CRC
  frame[4] = crc8(frame, 4);
  
}

void decodeSensorFrame(struct lacross_message *msg, byte *data, int data_len) {
  msg->isValid = true;

  msg->crc = crc8(data, 4);
  if (msg->crc != data[4]) {
    if (DEBUG) { Serial.println("## CRC FAIL ##"); }
    msg->isValid = false;
  }

  msg->id = 0;
  msg->id |= (data[0] & 0xF) << 2;
  msg->id |= (data[1] & 0xC0) >> 6;

  msg->header = (data[0] & 0xF0) >> 4;
  if (msg->header != 9) {
    if (DEBUG) { Serial.println("## UNSUPPORTED START ##"); }
    msg->isValid = false;
  }

  msg->newBatteryFlag = (data[1] & 0x20) >> 5;
  
  msg->bit12 = (data[1] & 0x10) >> 4;
  
  byte bcd[3];
  bcd[0] = data[1] & 0xF;
  bcd[1] = (data[2] & 0xF0) >> 4;
  bcd[2] = (data[2] & 0xF);
  msg->temperature = calculate_temperature(bcd);

  msg->weakBatteryFlag = (data[3] & 0x80) >> 7;

  msg->humidity = data[3] & 0b01111111;
}

void analyze_sensor_frame(byte *data, int data_len) {
  struct lacross_message msg;
  decodeSensorFrame(&msg, data, FRAME_LENGHT);

  byte filter[5];
  filter[0] = 0;
  filter[1] = 0;
  filter[2] = 0;
  filter[3] = 0;
  filter[4] = 0;

  bool hideIt = false;
  for (int f = 0; f < 5; f++) {
    if (msg.id == filter[f]) {
      hideIt = true;
      break;
    }
  }

  if (!hideIt) {
    // MilliSeconds
    static unsigned long lastMillis;
    unsigned long now = millis();
    char div[16];
    sprintf(div, "%06d ", now - lastMillis);
    lastMillis = millis();
    Serial.print(div);

    // Show the raw data bytes
    Serial.print("[");
    for (int i = 0; i < FRAME_LENGHT; i++) {
      Serial.print(data[i], DEC);
      Serial.print(" ");
    }
    Serial.print("]");

    // Check CRC
    if (!msg.isValid) {
      Serial.print(" CRC:WRONG");
    }
    else {
      Serial.print(" CRC:OK");

      // Start
      Serial.print(" S:");
      Serial.print(msg.header, DEC);

      // Sensor ID
      Serial.print(" ID:");
      Serial.print(msg.id, DEC);

      // New battery flag
      Serial.print(" NewBatt:");
      Serial.print(msg.newBatteryFlag, DEC);

      // Bit 12
      Serial.print(" Bit12:");
      Serial.print(msg.bit12, DEC);

      // Temperature
      Serial.print(" Temp:");
      Serial.print(msg.temperature);

      // Weak battery flag
      Serial.print(" WeakBatt:");
      Serial.print(msg.weakBatteryFlag, DEC);

      // Humidity
      Serial.print(" Hum:");
      Serial.print(msg.humidity, DEC);

      // CRC
      Serial.print(" CRC:");
      Serial.print(msg.crc, DEC);
    }

    Serial.println();
  }

}

//- helpText ---------------------------------------------------------------------------------------
const char helpText[] PROGMEM =
"\n"
"Available commands:" "\n"
"  <n> a                    - activity LED (0=off, 1=on)" "\n"
"  <n> d                    - DEBUG mode (0=suppress TX and bad packets)" "\n"
"  <n> r                    - data rate (0=17.241 kbps, 1=9.579 kbps)" "\n"
"  <n> t                    - toggle data rate intervall (0=no toggle, >0=seconds)" "\n"
"  <n> v                    - version and configuration report" "\n"
"  <n> x                    - used for tests" "\n"
"  <t10>,<t1>,<t0>,<hum>c   - set temperature and humidity for transmit" "\n"
"  <id>,<int>,<nbt>,<dr>i   - set the parameters for the transmit loop" "\n"
"  b1,b2,b3,b4s             - send the passed bytes plus the calculated CRC"
;

//- showString -------------------------------------------------------------------------------------
static void showString(PGM_P s) {
  for (;;) {
    char c = pgm_read_byte(s++);
    if (c == 0)
      break;
    if (c == '\n')
      Serial.print('\r');
    Serial.print(c);
  }
}

//- showHelp ---------------------------------------------------------------------------------------
static void showHelp() {
  Serial.print("\n[");
  Serial.print(PROGNAME);
  Serial.print('.');
  Serial.print(PROGVERS);
  Serial.println(']');
  showString(helpText);
  Serial.println();
}

static void handleSerialPort(char c) {
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
      DEBUG = value;
      break;
    case 'x':
      // Tests
      HandleCommandX(value);
      break;
    case 'a': 
      // Activity LED    
      ENABLE_ACTIVITY_LED = value;
      break;
    case 'r':     
      // Data rate
      if (value)
        DATA_RATE = DATA_RATE_9;
      else
        DATA_RATE = DATA_RATE_17;
      rf12_xfer(DATA_RATE);
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
      handleCommandS(commandData, ++commandDataPointer);
      commandDataPointer = 0;
      break;

    case 'i':
      commandData[commandDataPointer] = value;
      handleCommandI(commandData, ++commandDataPointer);
      commandDataPointer = 0;
      break;

    case 'c':
      commandData[commandDataPointer] = value;
      handleCommandC(commandData, ++commandDataPointer);
      commandDataPointer = 0;
      break;

    default:
      showHelp();
      break;
    }
    value = 0;
  }
  else if (' ' < c && c < 'A')
    showHelp();
}

void handleCommandS(byte *data, byte size) {
  if (size == 4){
    StopReceiver();

    // Calculate the CRC
    data[FRAME_LENGHT - 1] = crc8(data, FRAME_LENGHT - 1);

    rf12_sendArray(data, FRAME_LENGHT);

    StartReceiver();
  }
}

void handleCommandI(byte *values, byte size){
  // 14,43,20,0i  -> ID 14, Interval 4.3 Seconds, reset NewBatteryFlagafter 20 minutes, 17.241 kbps
  if (size == 4){
    transmitData.id = values[0];
    transmitData.interval = values[1] * 100;
    transmitData.newBatteryFlag = true;
    transmitData.newBatteryFlagResetTime = values[2] * 60000 + millis();
    transmitData.dataRate = values[3];
    transmitData.enabled = true;
  }
  else if (size == 1 && values[0] == 0){
    transmitData.enabled = false;
  }
  
}

void handleCommandC(byte *values, byte size){
  // 2,1,9,44c    -> Temperatur  21,9°C and 44% humidity
  // 129,4,5,77c  -> Temperatur -14,5°C and 77% humidity
  // To set a negative temperature set bit 7 in the first byte (add 128)
  if (size == 4){
    transmitData.temperature = (values[0] & 0b0111111) * 10 + values[1] + values[2] * 0.1;
    if (values[0] & 0b10000000) {
      transmitData.temperature *= -1;
    }
    
    transmitData.humidity = values[3];
  }
}

// This function is for testing 
void HandleCommandX(byte value) {
  lacross_message msg;
  msg.id = 20;
  msg.newBatteryFlag = true;
  msg.bit12 = false;
  msg.temperature = value;
  msg.weakBatteryFlag = false;
  msg.humidity = value;

  rf12_sendLacrossMessage(msg);
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
    Serial.print(DATA_RATE == DATA_RATE_9 ? "9.579 kbps" : "17.241 kbps");
  }
  Serial.println(']');
}


// **********************************************************************
void loop(void) {
  
  // Handle the commands from the serial port
  // ----------------------------------------
  if (Serial.available()) {
    handleSerialPort(Serial.read());
  }

  // Handle the data rate
  // --------------------
  if (TOGGLE_DATA_RATE > 0) {
    // After about 50 days millis() will overflow to zero 
    if (millis() < lastToggle) {
      lastToggle = 0;
    }

    if (millis() > lastToggle + TOGGLE_DATA_RATE * 1000) {
      if (DATA_RATE == DATA_RATE_9) {
        DATA_RATE = DATA_RATE_17;
      }
      else {
        DATA_RATE = DATA_RATE_9;
      }

      rf12_xfer(DATA_RATE);
      lastToggle = millis();

    }
  }

  // Priodically transmit
  // --------------------
  if (transmitData.enabled && millis() >= transmitData.lastTransmit + transmitData.interval) {
    transmitData.lastTransmit = millis();

    // Reset the NewBatteryFlag, if it's time to do it 
    if (transmitData.newBatteryFlag && millis() >= transmitData.newBatteryFlagResetTime) {
      transmitData.newBatteryFlag = false;
    }
    
    // Build a message with the current transmit data
    lacross_message msg;
    msg.id = transmitData.id;
    msg.newBatteryFlag = transmitData.newBatteryFlag;
    msg.bit12 = false;
    msg.temperature = transmitData.temperature;
    msg.weakBatteryFlag = false;
    msg.humidity = transmitData.humidity;

    // Set the data rate and send it
    int currentDataRate = DATA_RATE;
    rf12_xfer(transmitData.dataRate ? DATA_RATE_9 : DATA_RATE_17);
    rf12_sendLacrossMessage(msg);
    rf12_xfer(currentDataRate);
    DATA_RATE = currentDataRate;

    ledBlink(2);
  }

  // Handle the data reception
  // -------------------------
  if (RECEIVER_ENABLED) {
    if (receiveBufferPointer >= FRAME_LENGHT) {
      receiveBufferPointer = 0;
      StartReceiver();
    }

    if (rf12_fifoHasData()) {
      receiveBuffer[receiveBufferPointer++] = rf12_xfer(0xB000);

      if (receiveBufferPointer == FRAME_LENGHT) {
        if(ANALYZE_FRAMES) {
          analyze_sensor_frame(receiveBuffer, FRAME_LENGHT);
        }
        else {
          StopReceiver();
          ledBlink(1);

          if (DEBUG) {
            Serial.print("End receiving, HEX raw data: ");
            for (int i = 0; i < FRAME_LENGHT; i++) {
              Serial.print(receiveBuffer[i], HEX);
              Serial.print(" ");
            }
            Serial.println();
          }
          
          struct lacross_message msg;
          decodeSensorFrame(&msg, receiveBuffer, FRAME_LENGHT);
          if (msg.isValid) {
            String out = prepare_jeestring(&msg);

            if (out != "") {
              Serial.println(out);
            }
            Serial.flush();
          }
        }
      }
    }
  }
 
}


void StartReceiver() {
  if (DEBUG) { 
    Serial.println("\nStart receiver"); 
  }

  rf12_xfer(0x82C8);			// receiver on
  rf12_xfer(0xCA81);			// set FIFO mode
  rf12_xfer(0xCA83);			// enable FIFO

  // Ensure that the FIFO is empty
  for (byte i = 0; i < 10; i++) {
    rf12_xfer(0xB000);
  }
  
}

void StopReceiver() {
  if (DEBUG) {
    Serial.println("Stop receiver");
  }
  rf12_xfer(0x8208);			// Receiver off 
}



//prepares fixed-length output message for FHEM
String prepare_jeestring(struct lacross_message *msg) {
  // Format
  //
  // OK 9 56 1   4   156 37     ID = 56  T: 18.0  H: 37  no NewBatt
  // OK 9 49 1   4   182 54     ID = 49  T: 20.6  H: 54  no NewBatt
  // OK 9 55 129 4 192 56       ID = 55  T: 21.6  H: 56  WITH NewBatt 
  // OK 9 ID XXX XXX XXX XXX
  // |  | |  |   |   |   |
  // |  | |  |   |   |   --- Humidity incl. WeakBatteryFlag
  // |  | |  |   |   |------ Temp * 10 + 1000 LSB
  // |  | |  |   |---------- Temp * 10 + 1000 MSB
  // |  | |  |-------------- Sensor type (1 or 2) +128 if NewBatteryFlag
  // |  | |----------------- Sensor ID
  // |  |------------------- fix "9"
  // |---------------------- fix "OK"

  String pBuf;
  pBuf += "OK 9 ";
  pBuf += msg->id;
  pBuf += ' ';

  // bogus check humidity + eval 2 channel TX25IT
  // TBD .. Dont understand the magic here!?
  if ((msg->humidity >= 0 && msg->humidity <= 99)
    || msg->humidity == 106
    || (msg->humidity >= 128 && msg->humidity <= 227)
    || msg->humidity == 234) {
    pBuf += msg->newBatteryFlag ? 129 : 1;
    pBuf += ' ';
  }
  else if (msg->humidity == 125 || msg->humidity == 253) {
    pBuf += 2 | msg->newBatteryFlag ? 130 : 2;
    pBuf += ' ';
  }
  else {
    return "";
  }

  // add temperature
  uint16_t pTemp = (uint16_t)(msg->temperature * 10 + 1000);
  pBuf += (byte)(pTemp >> 8);
  pBuf += ' ';
  pBuf += (byte)(pTemp);
  pBuf += ' ';

  // bogus check temperature
  if (msg->temperature >= 60 || msg->temperature <= -40)
    return "";

  // add humidity
  byte hum = msg->humidity;
  if (msg->weakBatteryFlag) {
    hum |= 0x80;
  }
  pBuf += hum;

  return pBuf;
}


static void activityLed(byte on) {
  byte LED_PIN = 9;
  if (ENABLE_ACTIVITY_LED) {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, !on);
  }
}

static void ledBlink(unsigned int blinksCount) {
  if (blinksCount > 0) {
    if (blinksCount > 10) {
      blinksCount = 10; //max. 10 blinks are allowed
    }
    for (int i = 0; i<blinksCount; i++) {
      activityLed(1);
      delay(50);
      activityLed(0);
      delay(50);
    }

  }
}

//init
void setup(void) {
  lastToggle = millis();
  transmitData.enabled = false;

  Serial.begin(57600);
  if (DEBUG) {
    Serial.println("*** LaCrosse weather station wireless receiver for IT+ sensors ***");
  }
  rf12_la_init();
  if (DEBUG) {
    Serial.println("Radio setup complete. Starting to receive messages");
  }
  ledBlink(5);
}



// ******** SPI + RFM 12B functions ************************************************************************
// -- Board settings ---------------------------------------------------------
#define RF_PORT	PORTB
#define RF_DDR	DDRB
#define RF_PIN	PINB
#define SDI 3             // polling  4 Pins ,3 SPI + Chipselect
#define SPI_SCK 5         // differs with Jee-node lib !
#define CS  2  
#define SDO 4 


#define clrb(sfr, bit)     (_SFR_BYTE(sfr) &= ~_BV(bit)) 
#define setb(sfr, bit)     (_SFR_BYTE(sfr) |= _BV(bit)) 

unsigned short rf12_xfer(unsigned short value) {
  uint8_t i;

  clrb(RF_PORT, CS);
  for (i = 0; i<16; i++) {
    if (value & 32768)
      setb(RF_PORT, SDI);
    else
      clrb(RF_PORT, SDI);
    value <<= 1;
    if (RF_PIN&(1 << SDO))
      value |= 1;
    setb(RF_PORT, SPI_SCK);
    asm("nop");
    asm("nop");
    clrb(RF_PORT, SPI_SCK);
  }
  setb(RF_PORT, CS);
  return value;
}

void rf12_waitForTransmitRegister() {
  while (!(rf12_xfer(0x0000) & 0x8000)) {}
}

void rf12_sendByte(byte data) {
  rf12_waitForTransmitRegister();
  rf12_xfer(0xB800 | data);
}

void rf12_sendArray(byte *data, byte length) {
  if (DEBUG) {
    Serial.print("Sending data: ");
    for (int i = 0; i < length; i++) {
      Serial.print(data[i], DEC);
      Serial.print(" ");
    }
    Serial.println();
  }

  // Transmitter on
  rf12_xfer(0x8238);

  // Sync, sync, sync ...
  rf12_sendByte(0xAA);
  rf12_sendByte(0xAA);
  rf12_sendByte(0xAA);
  rf12_sendByte(0x2D);
  rf12_sendByte(0xD4);

  // Send the data
  for (int i = 0; i < length; i++) {
    rf12_sendByte(data[i]);
  }

  // Transmitter off
  _delay_ms(5);
  rf12_xfer(0x8208);
}

void rf12_sendLacrossMessage(lacross_message msg) {
  if (DEBUG) {
    Serial.print("TX: T=");
    Serial.print(msg.temperature);
    Serial.print(" H=");
    Serial.print(msg.humidity);
    Serial.print(" NB=");
    Serial.print(msg.newBatteryFlag);
    Serial.println();
  }

  byte frame[FRAME_LENGHT];
  encodeSensorFrame(msg, frame);
  rf12_sendArray(frame, FRAME_LENGHT);
}

bool rf12_fifoHasData() {
  bool result = false;
  clrb(RF_PORT, CS);
  asm("nop");
  asm("nop");
  if (RF_PIN&(1 << SDO)) {
    result = true;
  }
  setb(RF_PORT, CS);

  return result;
}

//radio settings for IT+ sensor (868.300MHz, FSK)
static void rf12_la_init() {
  RF_DDR = (1 << SDI) | (1 << SPI_SCK) | (1 << CS);
  RF_PORT = (1 << CS);
  for (uint8_t i = 0; i<10; i++) _delay_ms(10); // wait until POR done
  rf12_xfer(0x80E8);    // 80e8 CONFIGURATION EL,EF,868 band,12.5pF      // iT+ 915  80f8
  rf12_xfer(0xA67c);    // a67c FREQUENCY SETTING 868.300                // a67c = 915.450 MHz
  rf12_xfer(DATA_RATE); // DATA RATE
  rf12_xfer(0xC26a);    // c26a DATA FILTER COMMAND 
  rf12_xfer(0xCA12);    // ca12 FIFO AND RESET  8,SYNC,!ff,DR 
  rf12_xfer(0xCEd4);    // ced4 SYNCHRON PATTERN  0x2dd4 
  rf12_xfer(0xC49f);    // c49f AFC during VDI HIGH +15 -15 AFC_control_commAND
  rf12_xfer(0x94a0);    // 94a0 RECEIVER CONTROL VDI Medium 134khz LNA max DRRSI 103 dbm  
  rf12_xfer(0xCC77);    // cc77 not in RFM01 
  rf12_xfer(0x9850);    // Deviation 90 kHz 
  rf12_xfer(0xE000);    // e000 NOT USE 
  rf12_xfer(0xC800);    // c800 NOT USE 
  rf12_xfer(0xC040);    // c040 1.66MHz,2.2V 
}
