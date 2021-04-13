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

void setup()
{
    // Serial.begin(250000);

    pinMode(analogPin, INPUT);

    // Initalize I2C communication
    Wire.begin();

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
