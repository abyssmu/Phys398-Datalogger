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

    bool init()
    {
      if (!bme.begin()) return false;

      // Set up oversampling and filter initialization
      bme.setTemperatureOversampling(BME680_OS_8X);
      bme.setHumidityOversampling(BME680_OS_2X);
      bme.setPressureOversampling(BME680_OS_4X);
      bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
      bme.setGasHeater(320, 150); // 320*C for 150 ms

      return true;
    }
    void printBME()
    {
      if (!bme.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
      }
      Serial.print("Temperature = ");
      Serial.print(bme.temperature);
      Serial.println(" *C");
    
      Serial.print("Pressure = ");
      Serial.print(bme.pressure / 100.0);
      Serial.println(" hPa");
    
      Serial.print("Humidity = ");
      Serial.print(bme.humidity);
      Serial.println(" %");
    
      Serial.print("Gas = ");
      Serial.print(bme.gas_resistance / 1000.0);
      Serial.println(" KOhms");
    
      Serial.print("Approx. Altitude = ");
      Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
      Serial.println(" m");
    
      Serial.println();
      delay(1000);
    }
};
