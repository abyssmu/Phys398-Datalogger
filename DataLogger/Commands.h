#include "BMEwrapper.h"
#include "RTCwrapper.h"
#include "SDwrapper.h"

#define READ "1#"
#define WRITE "2#"
#define DELETE "3#"
#define CLEAR "***"

int checkCommand(String comm);
void runCommand(String& comm, BMEwrapper bme, RTCwrapper rtc, SDwrapper sd);
