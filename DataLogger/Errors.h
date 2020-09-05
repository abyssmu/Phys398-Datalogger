#define BMEerror 0
#define KEYPADerror 1
#define LCDerror 2
#define RTCerror 3
#define SDerror 4

void error(int errorCode)
{
  switch(errorCode)
  {
    case BMEerror:
      Serial.println("BME failed to start.");
      break;

    case KEYPADerror:
      Serial.println("Keypad failed to start.");
      break;

    case LCDerror:
      Serial.println("LCD failed to start.");
      break;

    case RTCerror:
      Serial.println("RTC failed to start.");
      break;

    case SDerror:
      Serial.println("SD failed to start.");
  }

  //Run endless loop to cause "crash"
  while (1);
}
