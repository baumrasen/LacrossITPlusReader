#include "Help.h"

//- helpText ---------------------------------------------------------------------------------------
const char helpText[] PROGMEM =
"\n"
"Available commands:" "\n"
"  <n>a                     - activity LED (0=off, 1=on)" "\n"
"  <n>d                     - DEBUG mode (0=suppress TX and bad packets)" "\n"
"  <n>h                     - height above sea level (m)" "\n"
"  <nnnnnn>f                - frequency (5 kHz steps e.g. 868315)" "\n"
"  <n>m                     - toggle mode (1: 17.241 kbps, 2: 9.579 kbps, 4: 8.842 kbps)" "\n"
"  <n>p                     - show raw payload data (0=off, 1=on, 2=only undecoded)" "\n"
"  <n>r                     - data rate (0: 17.241 kbps, 1: 9.579 kbps, 2: 8.842 kbps)" "\n"
"  <n>t                     - toggle data rate intervall (0=no toggle, >0=seconds)" "\n"
"  <n>v                     - version and configuration report" "\n"
"  <n>x                     - used for tests" "\n"
"  <n>y                     - Relay (0=no relay, 1=Relay received packets)" "\n"
"  <n>z                     - 1 = display analyzed frame data instead of normal data" "\n"
;


void Help::Show() {
  PGM_P s = helpText;
  for (;;) {
    char c = pgm_read_byte(s++);
    if (c == 0)
      break;
    if (c == '\n')
      Serial.print('\r');
    Serial.print(c);
  }
  Serial.println();
}


