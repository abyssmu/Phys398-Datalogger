#include "Commands.h"

CmdCenter cmdCenter;

//Not a lot goes on in the entry point.
//I took everything and put it in the CmdCenter class.
void setup() {
  Serial.begin(115200);
  #ifndef ESP8266
    while (!Serial); // wait for serial port to connect. Needed for native USB
  #endif

  cmdCenter.init();
}

void loop() {
  cmdCenter.getInput();
}
