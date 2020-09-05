#include "BMEwrapper.h"

bool BMEwrapper::init()
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

String BMEwrapper::collectBME()
{
  String data = "";

  readBME();

  data += String(bme.temperature) + ",";
  data += String(bme.pressure / 100) + ",";
  data += String(bme.humidity) + ",";
  data += String(bme.gas_resistance / 1000.0) + ",";
  data += String(bme.readAltitude(SEALEVELPRESSURE_HPA));

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

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
}

void BMEwrapper::readBME()
{
  if (!bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
}
