#include "BMEwrapper.h"

class CmdCenter
{
  private:
    //cmdand string to keep track of keypad codes
    String cmd = "";
    //Boolean cmdand to see if data can be logged
    bool logData = false;

  public:    
    CmdCenter() {}
    ~CmdCenter() {}

    bool init(); //initializes pins, classes, and modules
    int checkCmd(); //checks current command against list of commands
    void getInput(); //checks for any input from keypad and checks for commands
    bool runCmd(); //runs the commands obtained from checkCmd

    void collectData(); //function to run full data collection routine
};
