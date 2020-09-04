//SD headers
#include <SPI.h>
#include <SD.h>

class SDwrapper
{
  private:
    const int chipSelect = 53;

  public:
    SDwrapper() {}
    ~SDwrapper() {}

    bool init()
    {
      return SD.begin(chipSelect);
    }
    bool checkFile()
    {
      return SD.exists("datalog.txt");
    }
    void deleteSD()
    {
      SD.remove("datalog.txt");
    }
    void readSD()
    {
      File dataFile = SD.open("datalog.txt");
      
      if (dataFile) {
        while (dataFile.available()) {
          Serial.write(dataFile.read());
        }
        dataFile.close();
      }
    }
    void writeSD()
    {
      // make a string for assembling the data to log:
      String dataString = "";
    
      // read three sensors and append to the string:
      for (int analogPin = 0; analogPin < 3; analogPin++) {
        int sensor = analogRead(analogPin);
        dataString += String(sensor);
        if (analogPin < 2) {
          dataString += ",";
        }
      }
    
      // open the file. note that only one file can be open at a time,
      // so you have to close this one before opening another.
      File dataFile = SD.open("datalog.txt", FILE_WRITE);
    
      // if the file is available, write to it:
      if (dataFile) {
        dataFile.println("Hello, world!");
        dataFile.close();
      }
      // if the file isn't open, pop up an error:
      else {
        Serial.println("error opening datalog.txt");
      }
    }
};
