/*
    LiquidMenu adaption to Quiic LCD Display

    This is the get started example demonstrating how to create
    a menu of two screens with dynamically changing information.

    The first screen shows some static text. The second screen
    shows the reading of analog pin A1. To display a changing
    variable on the LCD, simply put the variable in the LiquidLine
    constructor. In this case the LiquidLine object is "analogReading_line"
    and the variable is "analogReading". This line is on the second
    screen. The value of the analog pin is read every second and if
    it has changed the display is updated with the new value. The
    menu cycles through its two screens every five seconds.

    This is adapted from the LiquidMenu library  - hello_menu.ino example to
    work with SparkFun Quiic LCD displays.

    The circuit:
    - Connect the Quiic LCD Screen to the bus

    Created July 24, 2016
        by Vasil Kalchev
    https://github.com/VasilKalchev/LiquidMenu
    
    Modified Apr 12, 2021
        by Alex Santiago

-------------------------------------- CONFIGURATION NOTE: --------------------------------------
    The LiquidMenu_config.h file needs to be modified in order to work with
    the QuiicSerLCD library. The following lines shall be commented:
    *   #define LIQUIDMENU_LIBRARY LiquidCrystal_LIBRARY
    *   #include <LiquidCrystal.h>
    *   #define DisplayClass LiquidCrystal

    The following lines shall be uncommented and changed from -> to:
    *   #include <LIBRARY_HEADER.h>              -> #include <SerLCD.h>
    *   #define DisplayClass LIBRARY_CONSTRUCTOR -> #define DisplayClass SerLCD

-------------------------------------------------------------------------------------------------
*/

#include <LiquidMenu.h> // Lib for Menu (https://github.com/VasilKalchev/LiquidMenu)
#include <Wire.h>       // For I2C Communication
#include <SerLCD.h>     // SparkFun Quiic LCD Library http://librarymanager/All#SparkFun_SerLCD
#include "MasterControl.h"

SerLCD lcd; // Initialize the library with default I2C address 0x72

/*
    Variable 'analogReading' is later configured to be printed on the display.'
    lastAnalogReading' is used to check if the variable has changed.
*/
const byte analogPin = A1;
unsigned short analogReading = 0;
unsigned short lastAnalogReading = 0;

/*
    Variables used for periodic execution of code. The first one is the period
    in milliseconds and the second one is the last time the code executed.
*/
unsigned int period_check = 1000;
unsigned long lastMs_check = 0;

unsigned int period_nextScreen = 5000;
unsigned long lastMs_nextScreen = 0;

/*
    LiquidLine objects represent a single line of text and/or variables
    on the display. The first two parameters are the column and row from
    which the line starts, the rest of the parameters are the text and/or
    variables that will be printed on the display. They can be up to four.
*/
// Here the line is set to column 1, row 0 and will print the passed
// string and the passed variable.
LiquidLine welcome_line1(1, 0, "LiquidMenu ", LIQUIDMENU_VERSION);
// Here the column is 3, the row is 1 and the string is "Hello Menu".
LiquidLine welcome_line2(3, 1, "Hello Menu");

/*
    LiquidScreen objects represent a single screen. A screen is made of
    one or more LiquidLine objects. Up to four LiquidLine objects can
    be inserted from here, but more can be added later in setup() using
    welcome_screen.add_line(someLine_object);
*/
// Here the LiquidLine objects are the two objects from above.
LiquidScreen welcome_screen(welcome_line1, welcome_line2);

// Here there is not only a text string but also a changing integer variable.
LiquidLine analogReading_line(0, 0, "Analog: ", analogReading);
LiquidScreen secondary_screen(analogReading_line);

/*
    The LiquidMenu object combines the LiquidScreen objects to form the
    menu. Here it is only instantiated and the screens are added later
    using menu.add_screen(someScreen_object);. This object is used to
    control the menu, for example: menu.next_screen(), menu.switch_focus()...
*/
LiquidMenu menu(lcd);

int16_t response[COMMAND_SIZE];

void setup()
{
    // Serial.begin(BAUD_RATE);
    pinMode(analogPin, INPUT);

    // Initalize I2C communication
    Wire.begin();
    Wire.onReceive(receiveCommand);
    // Wire.onRequest(onRequest);

    lcd.begin(Wire); //Set up the LCD for I2C communication

    // This is the method used to add a screen object to the menu.
    menu.add_screen(welcome_screen);
    menu.add_screen(secondary_screen);
}

void loop()
{
    // Periodic reading of the analog pin.
    if (millis() - lastMs_check > period_check)
    {
        lastMs_check = millis();
        analogReading = analogRead(analogPin);
        // Check if the analog value have changed and update the display if it has
        if (analogReading != lastAnalogReading)
        {
            lastAnalogReading = analogReading;
            menu.update();
        }
    }

    // Periodic switching to the next screen.
    if (millis() - lastMs_nextScreen > period_nextScreen)
    {
        lastMs_nextScreen = millis();
        menu.next_screen();
    }
}

void receiveCommand()
{
    // Read commands on the I2C bus
    int i = 0;
    while (Wire.available() && i <= COMMAND_SIZE)
    {
        response[i] += Wire.read();
        i++;
    }
}

void sendCommand(char _command, int16_t _value = 0)
{
    // Send commands to the coil winder with the corresponding value
    // Format answer as array and separate the integer into two bytes
    byte command[COMMAND_SIZE] = {_command, _value & 0x255, _value >> 8 & 0x255};

    Wire.beginTransmission(COIL_WINDER_I2C_ADDRESS);
    Wire.write(command, sizeof(command));
    Wire.endTransmission();
}

void requestData(byte _address, char _command, int16_t _response[COMMAND_SIZE])
{
    sendCommand(_command);

    // Process data requests from the coil winder
    Wire.requestFrom(_address, COMMAND_SIZE);

    response[COMMAND_SIZE] = {0};

    int i = 0;
    while (Wire.available() && i <= COMMAND_SIZE)
    {
        response[i] += Wire.read();
        i++;
    }

    // Convert the two bytes to an integer response
    response[COMMAND_SIZE - 1] = response[COMMAND_SIZE - 1] << 8 | response[COMMAND_SIZE];
    response[COMMAND_SIZE] = 0;
}

/*
  ?Command Instruction Set:
      (char)command (uint16_t)
    
    000 -> Print help message in serial
    001 -> Define Coil Width
    002 -> Define Coil Wire Gauge in mm
    003 -> Define Coil Turns
    010 -> Characterize Coil with saved parameters
    011 -> Build Coil with saved parameters
    020 -> Get saved coil_width
    021 -> Get saved wire_gauge_mm
    022 -> Get saved turns
    023 -> Get calculated number of layers
    024 -> Get calculated turns per layer
    025 -> Get calculated coil height
    030 -> Turn Steppers Off
    031 -> Turn Steppers On
    032 -> Home the feeder
    033 -> Get leading motor speed
    034 -> Get feeder position in steps
    035 -> Get spindle position in steps
    036 -> Get completed turns
    040 -> Change Feeder RPM
    041 -> Change Feeder ACCEL
    042 -> Change Feeder DECEL
    043 -> Change Feeder POLARITY
    044 -> Change Spindle RPM
    045 -> Change Spindle ACCEL
    046 -> Change Spindle DECEL
    047 -> Change Spindle POLARITY
    048 -> Change Microstepping Resolution
    050 -> Change feeder offset
    051 -> Change speed factor percentage
    051 -> Get speed factor percentage
    099 -> Load Default Setings
    117 -> Message
    118 -> Response with required data
*/