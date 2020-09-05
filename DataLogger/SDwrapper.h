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
    bool checkFile(String filename)
    {
      return SD.exists(filename);
    }
    void deleteSD(String filename)
    {
      SD.remove(filename);
    }
    void printSD(String filename)
    {
      File dataFile = SD.open(filename);
      
      if (dataFile) {
        while (dataFile.available()) {
          Serial.write(dataFile.read());
        }
        dataFile.close();
      }
    }
    void writeSD(String filename, String data)
    {    
      File dataFile = SD.open(filename, FILE_WRITE);
    
      if (dataFile) {
        dataFile.println(data);
        dataFile.close();
      }
      else {
        Serial.println("error opening datalog.txt");
      }
    }
};
