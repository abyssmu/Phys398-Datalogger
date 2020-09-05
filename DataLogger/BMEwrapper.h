#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#define SEALEVELPRESSURE_HPA (1013.25)

class BMEwrapper
{
  private:
    Adafruit_BME680 bme; // I2C

  public:
    BMEwrapper() {}
    ~BMEwrapper() {}

    bool init();
    String collectBME();
    void printBME();
    void readBME();
};
