#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#define SEALEVELPRESSURE_HPA (1013.25) //atmospheric pressure at sea level

//This is the class that controls the BME680.
class BMEwrapper
{
  private:
    Adafruit_BME680 bme; //I2C

  public:
    //the constructor and destructor are set to default
    BMEwrapper() {}
    ~BMEwrapper() {}

    bool init(); //initialized the bme680 to standard settings
    String collectBME(); //collects bme data in a comma seperated string
    void printBME(); //prints bme data to serial window
    void readBME(); //reads the pins connected to bme
};
