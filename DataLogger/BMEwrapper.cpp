#include "BMEwrapper.h"

bool BMEwrapper::init()
{
  if (!bme.begin()) return false;

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);

  return true;
}

String BMEwrapper::collectBME()
{
  String data = "";

  readBME();

  data += String(bme.temperature) + ",";
  data += String(bme.pressure / 100) + ",";
  data += String(bme.humidity);

  return data + '\n';
}

void BMEwrapper::printBME()
{
  readBME();
  
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  Serial.println();
}

void BMEwrapper::readBME()
{
  if (!bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
}
