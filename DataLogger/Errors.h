#define BMEerror 0
#define GPSerror 1
#define KEYPADerror 2
#define LCDerror 3
#define RTCerror 4
#define SDerror 5

//Debugging error function that outputs to serial monitor
void error(int errorCode)
{
  switch(errorCode)
  {
    case BMEerror:
      Serial.println("BME failed to start.");
      break;

    case GPSerror:
      Serial.println("GPS failed to start.");
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
