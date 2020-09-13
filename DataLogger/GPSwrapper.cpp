#include "GPSwrapper.h"

bool GPSwrapper::init()
{
  // 9600 baud is the default rate for the Ultimate GPS
  GPSSerial.begin(9600);

  return true;
}

String GPSwrapper::collectGPS()
{
  String data = readGPS();
  
  if (data == "@")
  {
    return "No data.\n";
  }

  return data;
}

void GPSwrapper::printGPS()
{
  String data = readGPS();
  
  if (data == "@")
  {
    Serial.print("No data.\n");
    return;
  }

  Serial.print(data);
}

String GPSwrapper::readGPS()
{
  String data = "";
  int lineEnds = 0;
  
  while(lineEnds < 2)
  {
    if (Serial.available()) {
      char c = Serial.read();
      GPSSerial.write(c);
    }
    if (GPSSerial.available()) {
      char c = GPSSerial.read();

      data += c;

      if (c == '\n')
      {
        ++lineEnds;
      }
    }
  }

  return data;
}
