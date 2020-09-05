#include "Commands.h"

CmdCenter cmdCenter;

void setup() {
  Serial.begin(9600);
  #ifndef ESP8266
    while (!Serial); // wait for serial port to connect. Needed for native USB
  #endif

  cmdCenter.init();
}

void loop() {
  cmdCenter.getInput();
}
