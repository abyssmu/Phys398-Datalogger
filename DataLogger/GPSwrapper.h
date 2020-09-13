#include <Arduino.h>

#define GPSSerial Serial2

class GPSwrapper
{
  public:
    GPSwrapper() {}
    ~GPSwrapper() {}

    bool init();
    String collectGPS();
    void printGPS();
    String readGPS();
};
