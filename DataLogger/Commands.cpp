#include "Commands.h"

int checkCommand(String comm)
{
  int clearSize = sizeof(CLEAR) - 1;
  if (comm == READ) return 1;
  if (comm == WRITE) return 2;
  if (comm == DELETE) return 3;

  if (comm.substring(comm.length() - clearSize) == CLEAR) return 99;

  return 0;
}

void runCommand(String& comm, BMEwrapper bme, RTCwrapper rtc, SDwrapper sd)
{
  switch(checkCommand(comm))
  {
    case 1:
      sd.readSD();
      break;

    case 2:
      sd.writeSD();
      break;

    case 3:
      sd.deleteSD();
      break;

    case 99:
      comm = "";
      break;
  }
}
