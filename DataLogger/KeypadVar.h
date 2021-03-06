#include <Keypad.h>

const byte ROWS = 4; //four rows
const byte COLS = 3; //four columns
//define the cymbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

//Womack pins
//byte rowPins[ROWS] = {A4, A5, A6, A7}; //connect to the row pinouts of the keypad
//byte colPins[COLS] = {A1, A2, A3}; //connect to the column pinouts of the keypad

//Logger pins
byte colPins[COLS] = {2, 3, 18};
byte rowPins[ROWS] = {31, 33, 35, 37};

//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
