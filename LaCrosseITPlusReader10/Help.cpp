#include "Help.h"

//- helpText ---------------------------------------------------------------------------------------
const char helpText[] PROGMEM =
"\n"
"Available commands:" "\n"
"  <n>a                     - activity LED (0=off, 1=on)" "\n"
"  <n>d                     - DEBUG mode (0=suppress TX and bad packets)" "\n"
"  <n>r                     - data rate (0=17.241 kbps, 1=9.579 kbps)" "\n"
"  <n>t                     - toggle data rate intervall (0=no toggle, >0=seconds)" "\n"
"  <n>v                     - version and configuration report" "\n"
"  <n>x                     - used for tests" "\n"
"  <t10>,<t1>,<t0>,<hum>c   - set temperature and humidity for transmit" "\n"
"  <id>,<int>,<nbt>,<dr>i   - set the parameters for the transmit loop" "\n"
"  b1,b2,b3,b4s             - send the passed bytes plus the calculated CRC"
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


