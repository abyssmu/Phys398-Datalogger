#include "RTClib.h"

class RTCwrapper
{
  private:
    RTC_DS3231 rtc;

  public:
    RTCwrapper() {}
    ~RTCwrapper() {}

    bool init();
    String collectRTC();
    void printRTC();
};
