/* 
!! Final-Project: Automatic Layer/Orthocyclic Coil Winder Implementing Linear Winding
  ELEN 447-131
  Prof. Miguel Goenaga
  
  Modified May 10, 2021
  by: Alex D. Santiago Vargas & Gretchell M. Hiraldo Martinez & Gabriel Vera Rodríguez

  This system controls the winding process according to the inputs received by the MasterControl
  The commands can be sent via any device with serial communication or I2C.


  ?Hardware Connections:
      From Circuit Pinout.docx

  ?Command Instruction Set:
      (int16_t)command (int16_t)value
    
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
    052 -> Get speed factor percentage
    099 -> Load Default Setings
    117 -> Message
    118 -> Response with required data

    Example:
      (int16_t)41 (int16_t)1500

  ---------------- TODO: ----------------
  * Add Pinout description from hardware/Schematics/Circuit Pinouts.docx in the intro commentary @ghiraldo5
  * Verify Command Instruction Set @ByteCommando
*/

// Libraries
#include <Arduino.h>
#include <Wire.h>
#include "CoilWinder.h"
#include "EEPROM.h"
#include "TeensyStep.h"
#include <math.h>
#include <vector>

// Stepper Motor Settings variables
int16_t FEEDER_RPM;
int16_t SPINDLE_RPM;
uint16_t FEEDER_ACCEL;
uint16_t FEEDER_DECEL;
uint16_t SPINDLE_ACCEL;
uint16_t FEEDER_OFFSET;
uint16_t SPINDLE_DECEL;
boolean FEEDER_POLARITY;
boolean SPINDLE_POLARITY;
uint8_t MICROSTEPS;
float SPEED_FACTOR;
uint32_t STEPS_PER_REV;
float FEEDER_STEPS_PER_MILIMETER;
uint8_t current_layer;
int16_t instruction[COMMAND_SIZE];
byte response[RESPONSE_SIZE];

// Stepper Driver Objects for feeder ans spindle
Stepper feeder(FEEDER_STEP_PIN, FEEDER_DIR_PIN);
Stepper spindle(SPINDLE_STEP_PIN, SPINDLE_DIR_PIN);

// Stepper Drivers Controller
StepControl step_controller(STEP_PULSE_WIDTH, SPEED_UPDATE_PERIOD); // Moving by position

// Limit flag
volatile bool homingRequired = false;

// Vectors to store stepper movements in steps. They are global so that they can be accessed by all the program
std::vector<int32_t> coil_feederSteps;
std::vector<int32_t> coil_spindleSteps;

// Store calculated coil parameters
uint32_t layers;
float turns;
float coil_width;
float wire_gauge_mm;
float turns_per_layer;
float calculated_height;

void setup()
{
  // Start serial communication
  delay(1000);
  Serial.begin(SERIAL_BAUD_RATE);

  // Start I2C Communication
  Wire.begin(COIL_WINDER_I2C_ADDRESS);
  Wire.onReceive(receiveCommand);
  Wire.onRequest(sendResponse);
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);

  // Load settings from the EEPROM memory to save when power is lost
  EEPROM.get(FEEDER_RPM_ADDRESS, FEEDER_RPM);
  EEPROM.get(SPINDLE_RPM_ADDRESS, SPINDLE_RPM);
  EEPROM.get(FEEDER_ACCEL_ADDRESS, FEEDER_ACCEL);
  EEPROM.get(FEEDER_DECEL_ADDRESS, FEEDER_DECEL);
  EEPROM.get(FEEDER_OFFSET_ADDRESS, FEEDER_OFFSET);
  EEPROM.get(SPINDLE_ACCEL_ADDRESS, SPINDLE_ACCEL);
  EEPROM.get(SPINDLE_DECEL_ADDRESS, SPINDLE_DECEL);
  EEPROM.get(FEEDER_POLARITY_ADDRESS, FEEDER_POLARITY);
  EEPROM.get(SPINDLE_POLARITY_ADDRESS, SPINDLE_POLARITY);
  EEPROM.get(MICROSTEPS_ADDRESS, MICROSTEPS);
  EEPROM.get(SPEED_FACTOR_ADDRESS, SPEED_FACTOR);

  STEPS_PER_REV = MOTOR_STEPS * MICROSTEPS;
  FEEDER_STEPS_PER_MILIMETER = STEPS_PER_REV / LEADSCREW_PITCH; // 1 revolution moves 2 mm

  // Micro switch as inputs with pullup resistors
  pinMode(END_STOP_2_PIN, INPUT_PULLUP);
  pinMode(END_STOP_1_PIN, INPUT_PULLUP);

  // Stepper pins as outputs
  pinMode(FEEDER_RST_PIN, OUTPUT);
  pinMode(SPINDLE_RST_PIN, OUTPUT);
  pinMode(FEEDER_EN_PIN, OUTPUT);
  pinMode(SPINDLE_EN_PIN, OUTPUT);
  pinMode(SPINDLE_SLP_PIN, OUTPUT);
  pinMode(FEEDER_SLP_PIN, OUTPUT);

  // Other pins
  pinMode(LED_BUILTIN, OUTPUT);

  // Load settings into stepper motor objects
  feeder
      //.setPullInSpeed(10)           // steps/s
      .setMaxSpeed(FEEDER_RPM * SPEED_FACTOR) // steps/s
      .setAcceleration(FEEDER_ACCEL)          // steps/s^2
      .setInverseRotation(FEEDER_POLARITY);   // Software will run stepper forward
                                              // .setStepPinPolarity(FEEDER_POLARITY);   // driver expects active high pulses

  spindle
      //.setPullInSpeed(10)           // steps/s
      .setMaxSpeed(SPINDLE_RPM * SPEED_FACTOR) // steps/s
      .setAcceleration(SPINDLE_ACCEL)          // steps/s^2
      .setInverseRotation(SPINDLE_POLARITY);   // Software will run stepper forward
                                               // .setStepPinPolarity(SPINDLE_POLARITY);   // driver expects active high pulses

  // Set the microstepping pins states
  setMicrostepping(MICROSTEPS, FEEDER_MS1_PIN, FEEDER_MS2_PIN, FEEDER_MS3_PIN);
  setMicrostepping(MICROSTEPS, SPINDLE_MS1_PIN, SPINDLE_MS2_PIN, SPINDLE_MS3_PIN);

  // loadDefaultSettings(); // Load Default Settings at startup; Comment to avoid

  // Attach falling hardware interrupts for end stops that are normally high, active low
  attachInterrupt(digitalPinToInterrupt(END_STOP_1_PIN), emergencyStop, FALLING);
  attachInterrupt(digitalPinToInterrupt(END_STOP_2_PIN), emergencyStop, FALLING);

  printHelp();
}

void loop()
{
  // Home the feeder if the flag is set
  if (homingRequired)
    feederHomming();

  // If data is incomming decode the instruction and execute accordingly
  if (Serial.available())
    decodeSerial();

  coilCharacterization(100, 1, 500, turns_per_layer, layers);
  buildCoil();
}

void loadDefaultSettings()
{
  // Write the default setting values into the EEPROM
  EEPROM.put(FEEDER_RPM_ADDRESS, (uint16_t)FEEDER_RPM_DEFAULT);
  EEPROM.put(SPINDLE_RPM_ADDRESS, (uint16_t)SPINDLE_RPM_DEFAULT);
  EEPROM.put(FEEDER_ACCEL_ADDRESS, (uint16_t)FEEDER_ACCEL_DEFAULT);
  EEPROM.put(FEEDER_DECEL_ADDRESS, (uint16_t)FEEDER_DECEL_DEFAULT);
  EEPROM.put(FEEDER_OFFSET_ADDRESS, (uint16_t)FEEDER_OFFSET_DEFAULT);
  EEPROM.put(SPINDLE_ACCEL_ADDRESS, (uint16_t)SPINDLE_ACCEL_DEFAULT);
  EEPROM.put(SPINDLE_DECEL_ADDRESS, (uint16_t)SPINDLE_DECEL_DEFAULT);
  EEPROM.put(FEEDER_POLARITY_ADDRESS, (bool)FEEDER_POLARITY_DEFAULT);
  EEPROM.put(SPINDLE_POLARITY_ADDRESS, (bool)SPINDLE_POLARITY_DEFAULT);
  EEPROM.put(MICROSTEPS_ADDRESS, (uint8_t)MICROSTEPS_DEFAULT);
  EEPROM.put(SPEED_FACTOR_ADDRESS, (uint8_t)SPEED_FACTOR_DEFAULT);
}

void setMicrostepping(uint16_t _Microstep, uint16_t _ms1_pin, uint16_t _ms2_pin, uint16_t _ms3_pin)
{

  /*
  The A4988 driver has the following options for microstepping:

*   ------------ Microstepping Table ------------
!!  MS1:    MS2:    MS3:    Microstep Resolution:
    Low	    Low	    Low	    Full step
    High    Low	    Low	    Half step
    Low     High    Low	    Quarter step
    High    High    Low	    Eighth step
    High    High    High    Sixteenth step
  */

  // Mask for the different microstepping pin states in MS3 MS2 MS1 order
  uint8_t MicrosteppingTable[] = {0b000, 0b001, 0b010, 0b011, 0b111};

  // Set all microstepping pins as outputs
  pinMode(_ms1_pin, OUTPUT);
  pinMode(_ms2_pin, OUTPUT);
  pinMode(_ms3_pin, OUTPUT);

  /* Select mask according to the microstep input
  
    If we calculate X by taking the log2 of Microstep = 2^X, we would have an index
    from 0 to 4 representing the all microstepping resolutions of the A4988. Hence we 
    input full (1), half (2), quarter (4), eighth (8), or sixteenth (16) resolution and
    we get the corresponding mask from element X of the table array.

    Example:
      For 16-Steps resolution we have:

      log2(16) = 4 => _microsteppingMask = MicrosteppingTable[4]
      => _microsteppingMask = 0b111
      
      Q.E.D. (quod erat demonstrandum) <=> TRIVIAL
  */
  uint8_t _microsteppingMask = MicrosteppingTable[uint8_t(log2(_Microstep))];

  /* Write the corresponfing mask bit to the microstepping pins

    We can select the corresponding output state by making a bitwise-AND with only the
    bit of the pin we want to write (MS1,MS2,MS3) as 1 and all others as 0. Then we shift
    to the right to move the state to be written to the first bit

    Example:
      For 16-Steps resolution, we have _microsteppingMask = 0b111

      => digitalWrite(_ms1_pin, (0b111 & 0b001) >> 0)
      => digitalWrite(_ms2_pin, (0b111 & 0b010) >> 1)
      => digitalWrite(_ms3_pin, (0b111 & 0b100) >> 2)

      => digitalWrite(_ms1_pin, 0b001 >> 0) 
      => digitalWrite(_ms2_pin, 0b010 >> 1) 
      => digitalWrite(_ms3_pin, 0b100 >> 2) 

      => digitalWrite(_ms1_pin, 1)
      => digitalWrite(_ms2_pin, 1)
      => digitalWrite(_ms3_pin, 1)

      Q.E.D. (quod erat demonstrandum) <=> TRIVIAL
  */
  digitalWrite(_ms1_pin, (_microsteppingMask & 0b001) >> 0);
  digitalWrite(_ms2_pin, (_microsteppingMask & 0b010) >> 1);
  digitalWrite(_ms3_pin, (_microsteppingMask & 0b100) >> 2);
}

void emergencyStop()
{
  /*
    When an end stop falls, activates this Interrupt Service Routine
    to innmediatly stop the winder no matter what, possibly missing steps
    from sudden braking. Hence it raises the homingRequired flag to home
    before running again
  */

  step_controller.emergencyStop();
  homingRequired = true;
}

void turnOnSteppers()
{
  /*
    !! Enable Pin:
    is active low input, when pulled LOW the A4988 driver is
    enabled. By default this pin is pulled low, unless you pull it HIGH.

    !! Sleep Pin:
    is active low input. Meaning, pulling this pin LOW puts the
    driver in sleep mode, minimizing the power consumption. You can invoke
    this especially when the motor is not in use 

    !! Reset Pin
    is an active low input, when pulled LOW all STEP inputs are
    ignored, until you pull it HIGH. It also resets the driver by setting
    the internal translator to a predefined Home state. Home state is the
    initial position from where the motor starts and depends on the
    microstep resolution.
  */

  digitalWrite(FEEDER_EN_PIN, LOW);   // Feeder Stepper Driver Enable
  digitalWrite(FEEDER_SLP_PIN, HIGH); // Feeder Stepper Driver Wake-Up (Un-Sleep)
  digitalWrite(FEEDER_RST_PIN, HIGH); // Feeder Stepper Driver Reset

  digitalWrite(SPINDLE_EN_PIN, LOW);   // Spindle Stepper Driver Enable
  digitalWrite(SPINDLE_SLP_PIN, HIGH); // Spindle Stepper Driver Wake-Up (Un-Sleep)
  digitalWrite(SPINDLE_RST_PIN, HIGH); // Spindle Stepper Driver Reset

  digitalWrite(LED_BUILTIN, HIGH); // BuiltIn LED On to indicate enabled steppers
}

void turnOffSteppers()
{
  /*
    !! Enable Pin:
    is active low input, when pulled LOW the A4988 driver is
    enabled. By default this pin is pulled low, unless you pull it HIGH.

    !! Sleep Pin:
    is active low input. Meaning, pulling this pin LOW puts the
    driver in sleep mode, minimizing the power consumption. You can invoke
    this especially when the motor is not in use 

    !! Reset Pin
    is an active low input, when pulled LOW all STEP inputs are
    ignored, until you pull it HIGH. It also resets the driver by setting
    the internal translator to a predefined Home state. Home state is the
    initial position from where the motor starts and depends on the
    microstep resolution.
  */

  digitalWrite(FEEDER_EN_PIN, HIGH); // Feeder Stepper Driver Disable
  digitalWrite(FEEDER_SLP_PIN, LOW); // Feeder Stepper Driver Sleep

  digitalWrite(SPINDLE_EN_PIN, HIGH); // Spindle Stepper Driver Disable
  digitalWrite(SPINDLE_SLP_PIN, LOW); // Spindle Stepper Driver Sleep

  digitalWrite(LED_BUILTIN, LOW); // BuiltIn LED Off to indicate disable steppers
}

void feederHomming()
{
  // Ensure all steppers are On and disable/put to sleep the spindle
  turnOnSteppers();
  digitalWrite(SPINDLE_EN_PIN, HIGH); // Spindle Stepper Driver Disable
  digitalWrite(SPINDLE_SLP_PIN, LOW); // Spindle Stepper Driver Sleep

  // Decrease the maximum speed and acceleration of the feeder by a factor
  feeder.setMaxSpeed(FEEDER_RPM / 4);
  feeder.setAcceleration(FEEDER_ACCEL / 4);

  // Read the state of the right end stop, its negated for programming logic because its active low
  bool rightLimit = !digitalRead(END_STOP_1_PIN);

  // Disable the interrupts during homing
  noInterrupts();
  cli();

  // Move to the right until the right end stop is activated
  while (!rightLimit)
  {
    feeder.setTargetRel(-480000);
    step_controller.moveAsync(feeder);
    rightLimit = !digitalRead(END_STOP_1_PIN);
  }

  // WHen is at the far most right, move step by step to the left until the end stop is deactivated
  while (rightLimit)
  {
    feeder.setTargetRel(1);
    step_controller.move(feeder);
    rightLimit = !digitalRead(END_STOP_1_PIN);
  }

  // Move an extra 5 steps as home position buffer
  feeder.setTargetRel(5);
  step_controller.move(feeder);

  // Turn off all the steppers to avoid movement during home position reset
  turnOffSteppers();

  // Pull down and then up to reset the feeder driver internal home state
  digitalWrite(FEEDER_RST_PIN, LOW); // Feeder Stepper Driver Reset
  delay(100);
  digitalWrite(FEEDER_RST_PIN, HIGH); // Feeder Stepper Driver Reset
  // Get the current position in software and set is as the new one
  feeder.setPosition(feeder.getPosition());

  // Enable the interrupts after homing
  interrupts();
  sei();

  // Set the feeder speed and acceleration to normal
  feeder.setMaxSpeed(FEEDER_RPM);
  feeder.setAcceleration(FEEDER_ACCEL);

  // Lower the homingRequired flag as it has homed
  homingRequired = false;
}

void coilCharacterization(float _coil_width = coil_width, float _wire_gauge_mm = wire_gauge_mm, float _turns = turns, float &_turns_per_layer = turns_per_layer, uint32_t &_layers = layers)
{
  // Set the global variables to the input values
  wire_gauge_mm = _wire_gauge_mm;
  coil_width = _coil_width;
  turns = _turns;
  // Calculate how many turns can be made for a wire of teh specified gauge and coil width
  _turns_per_layer = _coil_width / _wire_gauge_mm;

  // Calculate the total number of layers and if its a fraction of a total turn round up
  _layers = int32_t(_turns / _turns_per_layer);
  if (_turns_per_layer / _turns > _layers)
    _layers++;

  // Calculate the height of the coil layer by layer
  calculated_height = _layers * _wire_gauge_mm;
  // Print error if calculated height is bigger than established

  // Clear the motor step vectors
  coil_feederSteps.clear();
  coil_spindleSteps.clear();

  // Iterate the layers and assign the proper steps to the vectos
  for (uint32_t i = 0; i < _layers; i++)
  {
    // We move the feeder evenly from left to right for all the coil width in one direction per layer
    if (i % 2 == 0)
      // If the iteration is even means it is moving to the left, hence positive steps
      coil_feederSteps.push_back(int32_t(_coil_width * FEEDER_STEPS_PER_MILIMETER));
    else
      // If not even we are moving to the right, hence negative steps
      coil_feederSteps.push_back(int32_t(-1 * _coil_width * FEEDER_STEPS_PER_MILIMETER));

    // We rotate the spindle to wind all the estalished turns per layer
    if (i == (_layers - int32_t(1)))
      // If it is the last layer we only wind the remaining turns
      coil_spindleSteps.push_back(int32_t((_turns - (_layers - 1) * _turns_per_layer) * STEPS_PER_REV));
    else
      // For other layers we wind all the turns that can be fitted evenly into a layer
      coil_spindleSteps.push_back(int32_t(_turns_per_layer * STEPS_PER_REV));
  }
}

void buildCoil()
{
  // Starting delay
  delay(2000);

  // Home the feeder if needed
  if (homingRequired)
    feederHomming();
  // Iterate for all the layers and execute the steps established in the motor step vectors
  for (uint32_t i = 0; i < layers; i++)
  {
    current_layer = i + 1; // Set current layer

    // Set the steps from iteration element to the feeder and spindle
    feeder.setTargetRel(int32_t(coil_feederSteps[i]) + FEEDER_OFFSET);
    spindle.setTargetRel(int32_t(coil_spindleSteps[i]));
    // If no homing is required or the motors are executing a movement start the layer winding
    if (!homingRequired && !step_controller.isRunning())
    {
      // Move the motors in sync but without blocking other code execution
      step_controller.moveAsync(feeder, spindle);

      // Set timer delay
      uint16_t dt = 250;
      uint16_t prev_time = millis();

      // While there is no homming needed and the controller is executing a move, send the progress via serial
      while (!homingRequired && step_controller.isRunning())
      {
        if ((millis() - prev_time > dt))
        {
          sendProgress();
          prev_time = millis();
        }
      }
    }
    else
      break;

    delay(250);
  }
}

void decodeSerial()
{

  instruction[0] = (int16_t)Serial.readStringUntil(',').toInt();
  instruction[1] = (int16_t)Serial.readStringUntil(';').toInt();

  Serial.printf("\nReceived Command: %d with Value: %d\n", instruction[0], instruction[1]);
  decodeCommand(instruction[0], instruction[1]);
}

void decodeCommand(int16_t _command, int16_t _value)
{
  // Decode incoming info and execute the command accordingly
  switch (_command)
  {
  case (0):
    // 000 -> Print help message in serial
    printHelp();
    break;
  case (1):
    // 001 -> Define Coil Width
    coil_width = _value;
    break;
  case (2):
    // 002 -> Define Coil Wire Gauge in mm
    wire_gauge_mm = _value;
    break;
  case (3):
    // 003 -> Define Coil Turns
    turns = _value;
    break;
  case (10):
    // 010 -> Characterize Coil with saved parameters
    coilCharacterization();
    break;
  case (11):
    // 011 -> Build Coil with saved parameters
    buildCoil();
    break;
  case (20):
    // 020 -> Get saved coil_width
    setResponse(118, coil_width);
    break;
  case (21):
    // 021 -> Get saved wire_gauge_mm
    setResponse(118, wire_gauge_mm);
    break;
  case (22):
    // 022 -> Get saved turns
    setResponse(118, turns);
    break;
  case (23):
    // 023 -> Get calculated number of layers
    setResponse(118, layers);
    break;
  case (24):
    // 024 -> Get calculated turns per layer
    setResponse(118, turns_per_layer);
    break;
  case (25):
    // 025 -> Get calculated coil height
    setResponse(118, calculated_height);
    break;
  case (30):
    // 030 -> Turn Steppers Off
    turnOffSteppers();
    break;
  case (31):
    // 031 -> Turn Steppers On
    turnOnSteppers();
    break;
  case (32):
    // 032 -> Home the feeder
    feederHomming();
    break;
  case (33):
    // 033 -> Get leading motor speed
    setResponse(118, (uint16_t)step_controller.getCurrentSpeed()); // Send leading motor speed in X position with uint16_t data type
    break;
  case (34):
    // 034 -> Get feeder position in steps
    setResponse(118, (uint16_t)feeder.getPosition()); // Send leading motor speed in X position with uint16_t data type
    break;
  case (35):
    // 035 -> Get spindle position in steps
    setResponse(118, (uint16_t)spindle.getPosition()); // Send leading motor speed in X position with uint16_t data type
    break;
  case (36):
    // 036 -> Get completed turns
    setResponse(118, (uint16_t)(spindle.getPosition() / STEPS_PER_REV)); // Send spindle executed turns in X position with uint16_t data type
    break;
  case (40):
    // 040 -> Change Feeder RPM
    EEPROM.put(FEEDER_RPM_ADDRESS, (uint16_t)_value);
    EEPROM.get(FEEDER_RPM_ADDRESS, FEEDER_RPM);
    feeder.setMaxSpeed(FEEDER_RPM * SPEED_FACTOR); // steps/s
    break;
  case (41):
    // 041 -> Change Feeder ACCEL
    EEPROM.put(FEEDER_ACCEL_ADDRESS, (uint16_t)_value);
    EEPROM.get(FEEDER_ACCEL_ADDRESS, FEEDER_ACCEL);
    feeder.setAcceleration(FEEDER_ACCEL); // steps/s^2
    break;
  case (42):
    // 042 -> Change Feeder DECEL
    EEPROM.put(FEEDER_DECEL_ADDRESS, (uint16_t)_value);
    EEPROM.get(FEEDER_DECEL_ADDRESS, FEEDER_DECEL);
    break;
  case (43):
    // 043 -> Change Feeder POLARITY
    EEPROM.put(FEEDER_POLARITY_ADDRESS, (bool)_value);
    EEPROM.get(FEEDER_POLARITY_ADDRESS, FEEDER_POLARITY);
    feeder.setInverseRotation(FEEDER_POLARITY); // Software will run stepper forward
    break;
  case (44):
    // 044 -> Change Spindle RPM
    EEPROM.put(SPINDLE_RPM_ADDRESS, (uint16_t)_value);
    EEPROM.get(SPINDLE_RPM_ADDRESS, SPINDLE_RPM);
    spindle.setMaxSpeed(SPINDLE_RPM * SPEED_FACTOR); // steps/s
    break;
  case (45):
    // 045 -> Change Spindle ACCEL
    EEPROM.put(SPINDLE_ACCEL_ADDRESS, (uint16_t)_value);
    EEPROM.get(SPINDLE_ACCEL_ADDRESS, SPINDLE_ACCEL);
    spindle.setAcceleration(SPINDLE_ACCEL); // steps/s^2
    break;
  case (46):
    // 046 -> Change Spindle DECEL
    EEPROM.put(SPINDLE_DECEL_ADDRESS, (uint16_t)_value);
    EEPROM.get(SPINDLE_DECEL_ADDRESS, SPINDLE_DECEL);
    break;
  case (47):
    // 047 -> Change Spindle POLARITY
    feeder.setStepPinPolarity(SPINDLE_POLARITY_ADDRESS); // driver expects active high pulses
    EEPROM.get(SPINDLE_POLARITY_ADDRESS, SPINDLE_POLARITY);
    spindle.setInverseRotation(SPINDLE_POLARITY); // steps/s^2
    break;
  case (48):
    // 048 -> Change Microstepping Resolution
    EEPROM.put(MICROSTEPS_ADDRESS, (uint8_t)_value);
    EEPROM.get(MICROSTEPS_ADDRESS, MICROSTEPS);
    setMicrostepping(MICROSTEPS, FEEDER_MS1_PIN, FEEDER_MS2_PIN, FEEDER_MS3_PIN);
    setMicrostepping(MICROSTEPS, SPINDLE_MS1_PIN, SPINDLE_MS2_PIN, SPINDLE_MS3_PIN);
    break;
  case (50):
    // 050 -> Change feeder offset
    EEPROM.put(FEEDER_OFFSET_ADDRESS, (uint8_t)_value);
    EEPROM.get(FEEDER_OFFSET_ADDRESS, FEEDER_OFFSET);
    break;
  case (51):
    // 051 -> Change speed factor percentage
    if (_value > 100)
      _value = 100;
    else if (_value < 0)
      _value = 0;

    EEPROM.put(SPEED_FACTOR_ADDRESS, (float)_value / 100);
    EEPROM.get(SPEED_FACTOR_ADDRESS, SPEED_FACTOR);
    feeder.setMaxSpeed(FEEDER_RPM * SPEED_FACTOR);
    spindle.setMaxSpeed(SPINDLE_RPM * SPEED_FACTOR);
    break;
  case (52):
    // 052 -> Get speed factor percentage
    setResponse(118, int16_t(100 * SPEED_FACTOR)); // Send spindle executed turns in X position with uint16_t data type
    break;
  case (99):
    // 099 -> Load Default Setings
    loadDefaultSettings();
    break;
  case (117):
    // 117 -> Message
    // Command will be used when sending general information
    break;
  case (118):
    // 118 -> Response with required data
    // Command will be used when sending responses
    break;
  default:
    Serial.println(F("The command is not valid!\n"));
    printHelp();
    break;
  }
}

void printHelp()
{
  // Print help menu or ussage
  Serial.println(F("Command Instruction Set:"));
  Serial.println(F("\t(int16_t)command (int16_t)value"));

  Serial.println(F("0   -> Print help message in serial"));
  Serial.println(F("1   -> Define Coil Width"));
  Serial.println(F("2   -> Define Coil Wire Gauge in mm"));
  Serial.println(F("3   -> Define Coil Turns"));
  Serial.println(F("10  -> Characterize Coil with saved parameters"));
  Serial.println(F("11  -> Build Coil with saved parameters"));
  Serial.println(F("20  -> Get saved coil_width"));
  Serial.println(F("21  -> Get saved wire_gauge_mm"));
  Serial.println(F("22  -> Get saved turns"));
  Serial.println(F("23  -> Get calculated number of layers"));
  Serial.println(F("24  -> Get calculated turns per layer"));
  Serial.println(F("25  -> Get calculated coil height"));
  Serial.println(F("30  -> Turn Steppers Off"));
  Serial.println(F("31  -> Turn Steppers On"));
  Serial.println(F("32  -> Home the feeder"));
  Serial.println(F("33  -> Get leading motor speed"));
  Serial.println(F("34  -> Get feeder position in steps"));
  Serial.println(F("35  -> Get spindle position in steps"));
  Serial.println(F("36  -> Get completed turns"));
  Serial.println(F("40  -> Change Feeder RPM"));
  Serial.println(F("41  -> Change Feeder ACCEL"));
  Serial.println(F("42  -> Change Feeder DECEL"));
  Serial.println(F("43  -> Change Feeder POLARITY"));
  Serial.println(F("44  -> Change Spindle RPM"));
  Serial.println(F("45  -> Change Spindle ACCEL"));
  Serial.println(F("46  -> Change Spindle DECEL"));
  Serial.println(F("47  -> Change Spindle POLARITY"));
  Serial.println(F("48  -> Change Microstepping Resolution"));
  Serial.println(F("50  -> Change feeder offset"));
  Serial.println(F("51  -> Change speed factor percentage"));
  Serial.println(F("52  -> Get speed factor percentage"));
  Serial.println(F("99  -> Load Default Setings"));
  Serial.println(F("117 -> Message"));
  Serial.println(F("118 -> Response with required data"));
}

void sendProgress()
{
  // Get current controller speed from the faster motor
  int speed = step_controller.getCurrentSpeed();
  // Print the Feeder position and speed ratio as its slower
  Serial.printf("Feeder\n\tPosition: %d,\tSpeed: %d",
                feeder.getPosition(), (speed * (FEEDER_RPM / SPINDLE_RPM)));
  Serial.println();
  // setResponse(33, speed);
  // setResponse(34, feeder.getPosition());

  // Print the Spinde position and speed as its the leading motor
  Serial.printf("Spindle\n\tTurns: %d,\tSpeed: %d\n",
                spindle.getPosition() / STEPS_PER_REV, speed);
  Serial.println();
  // setResponse(35, spindle.getPosition());

  Serial.printf("Current Layer: %d", current_layer);
  Serial.printf("Completion: %d", (float)(turns / turns_per_layer));
  // setResponse(36, current_layer);
}

void receiveCommand(int numBytes)
{
  /*
    Read commands on the I2C bus
  */

  // Initialiaze array to receive the response in bytes
  for (int i = 0; i < RESPONSE_SIZE; i++)
    response[i] = 0;

  // Reads the inputs and saves it to the instruction buffer
  int i = 0;
  while (Wire.available() && i < RESPONSE_SIZE)
  {
    response[i] = Wire.read();
    i++;
  }

  // Converts the bytes to 16 bit integers and saves to the global response array
  instruction[0] = (response[0] << 8) | response[1];
  instruction[1] = (response[2] << 8) | response[3];
}

void setResponse(int16_t _command, int16_t _value = 0)
{
  // Prepare commands to the coil winder with the corresponding value

  // Convert the int16_t to byte and save to the response array
  response[0] = (_command >> 8) & 0xFF;
  response[1] = _command & 0xFF;
  response[2] = (_value >> 8) & 0xFF;
  response[3] = _value & 0xFF;

  // Save the plain values to the instruction array
  instruction[0] = _command;
  instruction[1] = _value;
}

void sendResponse()
{
  // Sends the prepared command
  Wire.write(response, sizeof(response));
}
