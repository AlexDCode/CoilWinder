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
    LiquidLine objects represent a single line of text and/or variables
    on the display. The first two parameters are the column and row from
    which the line starts, the rest of the parameters are the text and/or
    variables that will be printed on the display. They can be up to four.
*/
// Here the line is set to column 1, row 0 and will print the passed
// string and the passed variable.
LiquidLine welcome_line1(1, 0, "Coil Winder v1.0");
// Here the column is 3, the row is 2 and the string is "Hello Menu".
LiquidLine welcome_line2(3, 2, "May 12, 2021");

/*
    LiquidScreen objects represent a single screen. A screen is made of
    one or more LiquidLine objects. Up to four LiquidLine objects can
    be inserted from here, but more can be added later in setup() using
    welcome_screen.add_line(someLine_object);
*/
// Here the LiquidLine objects are the two objects from above.
LiquidScreen welcome_screen(welcome_line1, welcome_line2);

//Set up screem
LiquidLine setup_line_setuptitle(1, 0, "Set-Up");
LiquidLine setup_line_back(1, 1, "Back" );
LiquidLine setup_line_coil_width(1, 2, "Coil Width (mm): ");
LiquidLine setup_line_wire_gauge(1, 3, "Wire Gauge (mm): ");
LiquidLine setup_line_turns(1, 4, "Turns: ");
LiquidLine setup_line_start(1, 5, "Start");

LiquidScreen setup_screen(setup_line_setuptitle, setup_line_back, setup_line_coil_width, setup_line_wire_gauge, setup_line_turns, setup_line_start);

//Progress screen

LiquidLine progress_line_progress(1,0, "Progress Screen");
LiquidLine progress_line_percentage(1,1, "Progress %: ");
LiquidLine progress_line_stop(1,3, "Stop");

LiquidScreen progress_screen(progress_line_progress, progress_line_percentage, progress_line_stop);


/*
    The LiquidMenu object combines the LiquidScreen objects to form the
    menu. Here it is only instantiated and the screens are added later
    using menu.add_screen(someScreen_object);. This object is used to
    control the menu, for example: menu.next_screen(), menu.switch_focus()...
*/
LiquidMenu menu(lcd);

int16_t response[COMMAND_SIZE];
unsigned long lastMs;
int16_t dt = 5000;
int screen_number = 0;

void setup()
{
    // Serial.begin(BAUD_RATE);

    // Initalize I2C communication
    Wire.begin();
    Wire.onReceive(receiveCommand);

    lcd.begin(Wire); //Set up the LCD for I2C communication

    // This is the method used to add a screen object to the menu.
    menu.add_screen(welcome_screen);
    menu.add_screen(setup_screen);
    menu.add_screen(progress_screen);

    menu.change_screen(welcome_screen);

    delay(3000);
    lastMs = millis();
}

void loop()
{
    // Periodic reading of the analog pin.
    if (millis() - lastMs > dt)
    {
        if (screen_number == 0)
        {
            screen_number = 1;
            menu.change_screen(setup_screen);
        }
        else 
        {
            screen_number = 0;
            menu.change_screen(progress_screen);
        }

    }
}

void sendCommand(int16_t _command, int16_t _value = 0)
{
    /*
        Send commands to the coil winder with the corresponding value
    */

    // Converts the 16 bit integers to bytes
    byte instruction[] = {(_command >> 8) & 0xFF, _command & 0xFF, (_value >> 8) & 0xFF, _value & 0xFF};

    // Sends the instructiion
    Wire.beginTransmission(COIL_WINDER_I2C_ADDRESS);
    Wire.write(instruction, sizeof(instruction));
    Wire.endTransmission();
}

void requestData(byte _address, int16_t _command)
{
    /*
        Sends a command and reads the response
    */

    // Send the command
    sendCommand(_command);

    // Request the response
    Wire.requestFrom((uint8_t)_address, (uint8_t)(RESPONSE_SIZE));
}

void receiveCommand(int numBytes)
{
    /*
        Read commands on the I2C bus
    */

    // Initialiaze array to receive the response in bytes
    byte instruction[] = {0, 0, 0, 0};

    // Reads the inputs and saves it to the instruction buffer
    int i = 0;
    while (Wire.available() && i < RESPONSE_SIZE)
    {
        instruction[i] = Wire.read();
        i++;
    }

    // Converts the bytes to 16 bit integers and saves to the global response array
    response[0] = (instruction[0] << 8) | instruction[1];
    response[1] = (instruction[2] << 8) | instruction[3];
}

/*
  ?Command Instruction Set:
      (int16_t)command (int16_t)
    
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