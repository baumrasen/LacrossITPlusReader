// Tested with JeeLink v3 (2012-02-11)
// polling RFM12B to decode FSK iT+ with a JeeNode/JeeLink from Jeelabs.
// device  : iT+ TX29IT 04/2010 v36 D1     LA Crosse Technology (c) Europpe
//         : iT+ TX29DTH-IT 08/2008 v12 D1 LA Crosse Technology (c) Europe
//         : iT+ TX25U-IT                  LA Crosse Technology (c) USA
// info    : http://forum.jeelabs.net/node/110
//           http://fredboboss.free.fr/tx29/tx29_sw.php
//           http://www.f6fbb.org/domo/sensors/
//           http://www.mikrocontroller.net/topic/67273 benedikt.k org
//           rinie,marf,joop 1 nov 2011
//           slightly modified by Rufik (r.markiewicz@gmail.com)
//
// Changelog:
//  2012-02-11: initial release 1.0
//

#include <JeeLib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

#define clrb(sfr, bit)     (_SFR_BYTE(sfr) &= ~_BV(bit)) 
#define setb(sfr, bit)     (_SFR_BYTE(sfr) |= _BV(bit)) 

#define RF_PORT	PORTB
#define RF_DDR	DDRB
#define RF_PIN	PINB

#define SDI 3              // polling  4 Pins ,3 SPI + Chipselect
#define SPI_SCK 5              // differs with Jee-node lib !
#define CS  2  
#define SDO 4 

#define LED_PIN     9     // activity LED, comment out to disable
#define CRC_POLY    0x31  // CRC-8 = 0x31 is for: x8 + x5 + x4 + 1
#define DEBUG       0     // set to 1 to see debug messages

#define ENABLE_ACTIVITY_LED 1

/** 
 *	http://depa.usst.edu.cn/chenjq/www2/SDesign/JavaScript/CRCcalculation.htm
 *
 *	CRC order  (1..64): 8
 *	CRC polynom  (hex): 31
 *
 *	data in hex: 9EC5576A -> crc BD
 */

/* http://stackoverflow.com/questions/12015112/checksum-crc-calculation */
/// <summary>A variant of CRC-8</summary>
/// <param name="data">Pass here the first 4 bytes of data, e.g. { 0x4E 0x50 0x92 0x33 }</param>
/// <returns>The computed SRC value, e.g. 0xA1 for the data specified above.</returns>

/* 0x31 is for: x8 + x5 + x4 + 1 */
#define CRC_POLY 0x31
uint8_t crc8( uint8_t *data, int len )
{
  int i,j;
  uint8_t res = 0;
  for (j=0; j<len; j++) {
    uint8_t val = data[j];
    for( i = 0; i < 8; i++ ) {
      uint8_t tmp = (uint8_t)( ( res ^ val ) & 0x80 );
      res <<= 1;
      if( 0 != tmp ) {
        res ^= CRC_POLY;
      }
      val <<= 1;
    }
  }
  return res;
}


#define TEMP_OFFSET 40.0
float calculate_temperature(uint8_t *bcd)
{
  float t = 0;
  //t = (((bcd[0] * 100.0)+(bcd[1] * 10.0)+(bcd[2]))/10) - TEMP_OFFSET;
  t += bcd[0] * 100.0;
  t += bcd[1] * 10.0;
  t += bcd[2] * 1.0;
  t  = t/10;
  t -= TEMP_OFFSET;
  return t;
}

/**
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

#define FRAME_LENGHT 5
#define MSG_CRC_OFFSET 4
#define MSG_HEADER_MASK 0xF0
#define MSG_START 9
 
struct lacross_message {
  uint8_t header;
  uint8_t id;
  float   temp;
  uint8_t humidity;
  uint8_t batt_inserted;
};




int decode_sensor_frame(struct lacross_message *msg, uint8_t *data, int data_len)
{
  uint8_t bcd[3];

  if (crc8(data, 4) != data[MSG_CRC_OFFSET]) {
    if (DEBUG) { Serial.println("## CRC FAIL ##"); }
    return -1;
  }

  msg->header = (data[0] & MSG_HEADER_MASK) >> 4;
  if (msg->header != MSG_START) {
    if (DEBUG) { Serial.println("## UNSUPPORTED START ##"); }
	return -1;
  }

  msg->batt_inserted = (data[1] & 0x20) << 2;

  msg->id = 0;
  msg->id |= (data[0] & 0xF) << 2;
  msg->id |= (data[1] & 0xC0);

  bcd[0] = data[1] & 0xF;
  bcd[1] = (data[2] & 0xF0) >> 4;
  bcd[2] = (data[2] & 0xF);
  msg->temp = calculate_temperature(bcd);

  msg->humidity = data[3];
  return 0;
}


// **********************************************************************
void loop (void)
{
  unsigned char frame[FRAME_LENGHT];
  struct lacross_message msg;

  //get message
  receive(frame, FRAME_LENGHT);

  //check if valid for LaCrosse IT+ sensor
  if (decode_sensor_frame(&msg, frame, FRAME_LENGHT) == 0 ) {

    String out = prepare_jeestring(&msg);

    if (out != "")
      Serial.println(out);

    Serial.flush();
  }
  //simple delay, not really needed :)
  delay(100);
}


//receive message, data[] is length of 5
void receive(unsigned char *msg, int len)
{
  if (DEBUG) { Serial.println("Start receiving"); }

  rf12_rxdata(msg, len);
  //just blink once
  activityLed(1);
  delay(70);
  activityLed(0);

  if (DEBUG) {
    Serial.print("End receiving, HEX raw data: ");
    for (int i=0; i<len; i++) {
      Serial.print(msg[i], HEX);
      Serial.print(" ");
    }
    Serial.println();  	
  }
}


#define NO_HUMINITY_AVAILABLE 106
//prepares fixed-length output message for FHEM
//String prepare_jeestring(unsigned int sensor_id, float temperature, unsigned int humidity, unsigned int batInserted)
String prepare_jeestring(struct lacross_message *msg)
{

  // D:38: 23.7:99    // Robin - TX29-IT
  // D:28: 23.6:67    // Billy - TX29DTH-IT
  // D:FC: 24.8:99    // Billy - TX25IT CH1
  // D:FC: 30.9:99    // Billy - TX25IT CH2
  // D:1C: 24.3:62    // Billy - TX27TH-IT
  
  // OK 9 108 129 5 143 152
  // OK 9 20 1 4 205 106
  // OK 9 16 1 4 191 106
  // OK 9 248 1 4 131 106

  String     pBuf;
  bool       pValid = true;

  //----------------------------------------------------------------------------------------
  // even with CRC and bogus checks, the sketch might return to much false readings.
  // To filter for specific sensors, use a statement like this:
  //
  // if (sensor_id != 0xAA && sensor_id != 0xBB)
  //   return "";
  //----------------------------------------------------------------------------------------

  pBuf += "OK 9 ";
  pBuf += msg->id;
  pBuf += ' ';

  // bogus check humidity + eval 2 channel TX25IT
  // TBD .. Dont understand the magic here!?
  if ( (msg->humidity >= 0 && msg->humidity <= 99) 
       || msg->humidity == 106 
       || (msg->humidity >= 128 && msg->humidity <= 227) 
       || msg->humidity == 234)
  {
    pBuf += 1 | msg->batt_inserted;
    pBuf += ' ';
  } else if (msg->humidity == 125 || msg->humidity == 253 ) {
    pBuf += 2 | msg->batt_inserted;
    pBuf += ' ';
  } else {
    return "";
  }

  // add temperature
  uint16_t pTemp = (uint16_t)(msg->temp * 10 + 1000);
  pBuf  += (byte)(pTemp >> 8);
  pBuf  += ' ';
  pBuf  += (byte)(pTemp);
  pBuf  += ' ';

  // bogus check temperature
  if (msg->temp >= 60 || msg->temp <= -40)
    return "";

  // add humidity
  pBuf += msg->humidity;

  return pBuf;
}

static void activityLed (byte on)
{
  #if ENABLE_ACTIVITY_LED
#ifdef LED_PIN
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, !on);
#endif
  #endif
}

static void ledBlink(unsigned int blinksCount) {
  if (blinksCount > 0) {
    if (blinksCount > 10) {
      blinksCount = 10; //max. 10 blinks are allowed
    }
    for (int i=0; i<blinksCount; i++) {
      activityLed(1);
      delay(50);
      activityLed(0);
      delay(50);
    }

  }
}

//init
void setup (void) {
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

void rf12_rxdata(unsigned char *data, unsigned int number)
{	
  uint8_t  i;
  rf12_xfer(0x82C8);			// receiver on
  rf12_xfer(0xCA81);			// set FIFO mode
  rf12_xfer(0xCA83);			// enable FIFO
  for (i=0; i<number; i++)
  {	
    rf12_ready();
    *data++ = rf12_xfer(0xB000);
  }
  rf12_xfer(0x8208);			// Receiver off 
}

// ******** SPI + RFM 12B functies   ************************************************************************

unsigned short rf12_xfer(unsigned short value)
{	
  uint8_t i;

  clrb(RF_PORT, CS);
  for (i=0; i<16; i++)
  {	
    if (value&32768)
      setb(RF_PORT, SDI);
    else
      clrb(RF_PORT, SDI);
    value<<=1;
    if (RF_PIN&(1<<SDO))
      value|=1;
    setb(RF_PORT, SPI_SCK);
    asm("nop");
    asm("nop");
    clrb(RF_PORT, SPI_SCK);
  }
  setb(RF_PORT, CS);
  return value;
}


void rf12_ready(void)
{
  clrb(RF_PORT, CS);
  asm("nop");
  asm("nop");
  while (!(RF_PIN&(1<<SDO))); // wait until FIFO ready
  setb(RF_PORT, CS);
}

//radio settings for IT+ sensor (868.300MHz, FSK)
static void rf12_la_init() 
{
  RF_DDR=(1<<SDI)|(1<<SPI_SCK)|(1<<CS);
  RF_PORT=(1<<CS);
  for (uint8_t  i=0; i<10; i++) _delay_ms(10); // wait until POR done
  rf12_xfer(0x80E8); // 80e8 CONFIGURATION EL,EF,868 band,12.5pF      // iT+ 915  80f8 
  rf12_xfer(0xA67c); // a67c FREQUENCY SETTING 868.300                // a67c = 915.450 MHz
  rf12_xfer(0xC613); // c613 DATA RATE c613  17.241 kbps
  rf12_xfer(0xC26a); // c26a DATA FILTER COMMAND 
  rf12_xfer(0xCA12); // ca12 FIFO AND RESET  8,SYNC,!ff,DR 
  rf12_xfer(0xCEd4); // ced4 SYNCHRON PATTERN  0x2dd4 
  rf12_xfer(0xC49f); // c49f AFC during VDI HIGH +15 -15 AFC_control_commAND
  rf12_xfer(0x94a0); // 94a0 RECEIVER CONTROL VDI Medium 134khz LNA max DRRSI 103 dbm  
  rf12_xfer(0xCC77); // cc77 not in RFM01 
  rf12_xfer(0x9872); // 9872 transmitter not checked
  rf12_xfer(0xE000); // e000 NOT USE 
  rf12_xfer(0xC800); // c800 NOT USE 
  rf12_xfer(0xC040); // c040 1.66MHz,2.2V 
}

