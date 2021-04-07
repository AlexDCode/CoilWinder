/* 
!! Final-Project: Automatic Layer/Orthocyclic Coil Winder Implementing Linear Winding
  ELEN 447-131
  Prof. Miguel Goenaga
  
  Modified April 7, 2021
  by: Alex D. Santiago Vargas & Gretchell M. Hiraldo Martinez & Gabriel Vega Rodríguez

  This system controls the winding process according to the inputs received by the MasterController
  The commands can be sent via any device with serial communication.


  ?Hardware Connections:
      From Circuit Pinout.docx

  ?Command Instruction Set:
    (command_x, wire_gauge_mm, turns)

    C ==> Define Coil Parameters
      001 -> Define Coil Parameters (coil_width, wire_gauge_mm, turns)
      010 -> Build Coil with saved parameters
      020 -> Get saved coil_width
      021 -> Get saved wire_gauge_mm
      022 -> Get saved turns
      023 -> Get calculated number of layer
      024 -> Get calculated turns per layer

    M ==> Miscellaneous Data:
      001 -> Turn Steppers Off
      002 -> Turn Steppers On
      003 -> Home the feeder
      005 -> Get leading motor speed
      006 -> Get feeder (X) position in steps and spindle (Y) position in steps
      007 -> Get completed turns
      117 -> Message
      118 -> Response with required data

    S ==> Settings:
      000 -> Display Help Message (0, 0, 0)
      010 -> Change Feeder RPM (FEEDER_RPM, 0, 0)
      011 -> Change Feeder ACCEL (FEEDER_ACCEL, 0, 0)
      012 -> Change Feeder DECEL (FEEDER_DECEL, 0, 0)
      013 -> Change Feeder POLARITY (FEEDER_POLARITY, 0, 0)
      020 -> Change Spindle RPM (SPINDLE_RPM, 0, 0)
      021 -> Change Spindle ACCEL (SPINDLE_ACCEL, 0, 0)
      022 -> Change Spindle DECEL (SPINDLE_DECEL, 0, 0)
      023 -> Change Spindle POLARITY (SPINDLE_POLARITY, 0, 0)
      031 -> Change Microstepping Resolution (MICROSTEPS, 0, 0)
      099 -> Load Default Setings (0, 0, 0)

    Example:
      (char)S (uint8_t)10 (uint16_t)1000 (uint16_t)0 (uint16_t)0 (char)';'

  ---------------- TODO: ----------------
  * Add Pinout description from hardware/Schematics/Circuit Pinouts.docx in the intro commentary @ghiraldo5
  * Create Menu and Verify Command Instruction Set @ByteCommando
  * Verify parsing function to identify and execute commands @ByteCommando
  * Standarize how to send the progress and stats "G-Code" @ByteCommando | Done @AlexDCode
  * Create overrideSpeed() function to change the speed of the winding to a percentage of the established RPM using a potentiometer. @AlexDCode
  * Create overideFeederOffset() to change the feeder offset during execution @AlexDCode
*/

// Libraries
#include <Arduino.h>
#include "CoilWinder.h"
#include "EEPROM.h"
#include "TeensyStep.h"
#include <math.h>
#include <vector>

// Stepper Motor Settings variables
int32_t FEEDER_RPM;
int32_t SPINDLE_RPM;
uint32_t FEEDER_ACCEL;
uint32_t FEEDER_DECEL;
uint32_t SPINDLE_ACCEL;
uint32_t SPINDLE_DECEL;
boolean FEEDER_POLARITY;
boolean SPINDLE_POLARITY;
uint8_t MICROSTEPS;
uint32_t STEPS_PER_REV;
float FEEDER_STEPS_PER_MILIMETER;

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
  while (!Serial.available())
  {
    return;
  }

  // Load settings from the EEPROM memory to save when power is lost
  EEPROM.get(FEEDER_RPM_ADDRESS, FEEDER_RPM);
  EEPROM.get(SPINDLE_RPM_ADDRESS, SPINDLE_RPM);
  EEPROM.get(FEEDER_ACCEL_ADDRESS, FEEDER_ACCEL);
  EEPROM.get(FEEDER_DECEL_ADDRESS, FEEDER_DECEL);
  EEPROM.get(SPINDLE_ACCEL_ADDRESS, SPINDLE_ACCEL);
  EEPROM.get(SPINDLE_DECEL_ADDRESS, SPINDLE_DECEL);
  EEPROM.get(FEEDER_POLARITY_ADDRESS, FEEDER_POLARITY);
  EEPROM.get(SPINDLE_POLARITY_ADDRESS, SPINDLE_POLARITY);
  EEPROM.get(MICROSTEPS_ADDRESS, MICROSTEPS);

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
      .setMaxSpeed(FEEDER_RPM)             // steps/s
      .setAcceleration(FEEDER_ACCEL)       // steps/s^2
      .setInverseRotation(FEEDER_POLARITY) // Software will run stepper forward
      .setStepPinPolarity(HIGH);           // driver expects active high pulses

  spindle
      //.setPullInSpeed(10)           // steps/s
      .setMaxSpeed(SPINDLE_RPM)             // steps/s
      .setAcceleration(SPINDLE_ACCEL)       // steps/s^2
      .setInverseRotation(SPINDLE_POLARITY) // Software will run stepper forward
      .setStepPinPolarity(HIGH);            // driver expects active high pulses

  // Set the microstepping pins states
  setMicrostepping(MICROSTEPS, FEEDER_MS1_PIN, FEEDER_MS2_PIN, FEEDER_MS3_PIN);
  setMicrostepping(MICROSTEPS, SPINDLE_MS1_PIN, SPINDLE_MS2_PIN, SPINDLE_MS3_PIN);

  loadDefaultSettings(); // Load Default Settings at startup; Comment to avoid

  // Attach falling hardware interrupts for end stops that are normally high, active low
  attachInterrupt(digitalPinToInterrupt(END_STOP_1_PIN), righEndStop, FALLING);
  attachInterrupt(digitalPinToInterrupt(END_STOP_2_PIN), leftEndStop, FALLING);
}

void loop()
{
  // Home the feeder if the flag is set
  if (homingRequired)
    feederHomming();

  // If data is incomming decode the instruction and execute accordingly
  if (Serial.available() > 8)
    decodeSerial();
}

void loadDefaultSettings()
{
  // Write the default setting values into the EEPROM
  EEPROM.put(FEEDER_RPM_ADDRESS, (uint16_t)FEEDER_RPM_DEFAULT);
  EEPROM.put(SPINDLE_RPM_ADDRESS, (uint16_t)SPINDLE_RPM_DEFAULT);
  EEPROM.put(FEEDER_ACCEL_ADDRESS, (uint16_t)FEEDER_ACCEL_DEFAULT);
  EEPROM.put(FEEDER_DECEL_ADDRESS, (uint16_t)FEEDER_DECEL_DEFAULT);
  EEPROM.put(SPINDLE_ACCEL_ADDRESS, (uint16_t)SPINDLE_ACCEL_DEFAULT);
  EEPROM.put(SPINDLE_DECEL_ADDRESS, (uint16_t)SPINDLE_DECEL_DEFAULT);
  EEPROM.put(FEEDER_POLARITY_ADDRESS, (bool)FEEDER_POLARITY_DEFAULT);
  EEPROM.put(SPINDLE_POLARITY_ADDRESS, (bool)SPINDLE_POLARITY_DEFAULT);
  EEPROM.put(MICROSTEPS_ADDRESS, (uint8_t)MICROSTEPS_DEFAULT);
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

void righEndStop()
{
  /*
    When the right end stop falls, activates this Interrupt Service Routine
    to innmediatly stop the winder no matter what, possibly missing steps
    from sudden braking. Hence it raises the homingRequired flag to home
    before running again
  */

  step_controller.emergencyStop();
  homingRequired = true;
}

void leftEndStop()
{
  /*
    When the left end stop falls, activates this Interrupt Service Routine
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
      coil_feederSteps.push_back(int32_t(FEEDER_STEPS_PER_MILIMETER * _coil_width));
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
    // Set the steps from iteration element to the feeder and spindle
    feeder.setTargetRel(int32_t(coil_feederSteps[i]));
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
  // Read the serial into the data_buffer array until we reach the end character
  char data_buffer[9];
  Serial.readBytesUntil(';', data_buffer, sizeof(data_buffer));

  // Separate the command by its sections for parsing
  char command_char = data_buffer[0];
  uint8_t command_num = data_buffer[1];
  uint16_t command_x = (uint16_t)(data_buffer[2] << 16) | (uint16_t)data_buffer[3]; // If this conversion does not work, try inverting the array elements
  uint16_t command_y = (uint16_t)(data_buffer[4] << 16) | (uint16_t)data_buffer[5];
  uint16_t command_z = (uint16_t)(data_buffer[6] << 16) | (uint16_t)data_buffer[7];

  // Decode incoming info
  switch (command_char)
  {
  /*
    C ==> Define Coil Parameters
    001 -> Define Coil Parameters (coil_width, wire_gauge_mm, turns)
    010 -> Build Coil with saved parameters
    020 -> Get saved coil_width
    021 -> Get saved wire_gauge_mm
    022 -> Get saved turns
    023 -> Get calculated number of layer
    024 -> Get calculated turns per layer
  */
  case 'C':
    switch (command_num)
    {
    case (1):
      coilCharacterization(command_x, command_y, command_z);
      break;
    case (10):
      buildCoil();
      break;
    case (20):
      Serial.println(coil_width);
      break;
    case (21):
      Serial.println(wire_gauge_mm);
      break;
    case (22):
      Serial.println(turns);
      break;
    case (23):
      Serial.println(layers);
      break;
    case (24):
      Serial.println();
      break;
    default:
      printHelp(command_char, command_num);
      break;
    }

    break;

  /*
    M ==> Miscellaneous Data:
    001 -> Turn Steppers Off
    002 -> Turn Steppers On
    003 -> Home the feeder
    005 -> Get leading motor speed
    006 -> Get feeder (X) position in steps and spindle (Y) position in steps
    007 -> Get completed turns
    117 -> Message
    118 -> Response with required data
  */
  case 'M':
    switch (command_num)
    {
    case (1):
      turnOffSteppers();
      break;
    case (2):
      turnOnSteppers();
      break;
    case (3):
      feederHomming();
      break;
    case (5):
      Serial.print((char)'M');                                   // Send command group character with char data type
      Serial.print((uint8_t)118);                                // Send command number with uint8_t data type
      Serial.print((uint16_t)step_controller.getCurrentSpeed()); // Send leading motor speed in X position with uint16_t data type
      Serial.print((uint16_t)0);                                 // Fill with 0 the Y position as is not used with uint16_t data type
      Serial.print((uint16_t)0);                                 // Fill with 0 the Z position as is not used with uint16_t data type
      Serial.println(';');                                       // Send ending character ; with char data type
      break;
    case (6):
      Serial.print((char)'M');                       // Send command group character with char data type
      Serial.print((uint8_t)118);                    // Send command number with uint8_t data type
      Serial.print((uint16_t)feeder.getPosition());  // Send feeder position in X position with uint16_t data type
      Serial.print((uint16_t)spindle.getPosition()); // Send spindle position in X position with uint16_t data type
      Serial.print((uint16_t)0);                     // Fill with 0 the Z position as is not used with uint16_t data type
      Serial.println(';');                           // Send ending character ; with char data type
      break;
    case (7):
      Serial.print((char)'M');                                         // Send command group character with char data type
      Serial.print((uint8_t)118);                                      // Send command number with uint8_t data type
      Serial.print((uint16_t)(spindle.getPosition() / STEPS_PER_REV)); // Send spindle executed turns in X position with uint16_t data type
      Serial.print((uint16_t)0);                                       // Fill with 0 the Y position as is not used with uint16_t data type
      Serial.print((uint16_t)0);                                       // Fill with 0 the Z position as is not used with uint16_t data type
      Serial.println(';');                                             // Send ending character ; with char data type
      break;
    case (117):
      // Command will be used when sending general information
      break;
    case (118):
      // Command will be used when sending responses
      break;
    default:
      printHelp();
      break;
    }

    break;

  /*
    S ==> Settings:
    000 -> Display Help Message (0, 0, 0)
    010 -> Change Feeder RPM (FEEDER_RPM, 0, 0)
    011 -> Change Feeder ACCEL (FEEDER_ACCEL, 0, 0)
    012 -> Change Feeder DECEL (FEEDER_DECEL, 0, 0)
    013 -> Change Feeder POLARITY (FEEDER_POLARITY, 0, 0)
    020 -> Change Spindle RPM (SPINDLE_RPM, 0, 0)
    021 -> Change Spindle ACCEL (SPINDLE_ACCEL, 0, 0)
    022 -> Change Spindle DECEL (SPINDLE_DECEL, 0, 0)
    023 -> Change Spindle POLARITY (SPINDLE_POLARITY, 0, 0)
    031 -> Change Microstepping Resolution (MICROSTEPS, 0, 0)
    099 -> Load Default Setings (0, 0, 0)
  */
  case 'S':
    switch (command_num)
    {
    case (0):
      printHelp();
      break;
    case (10):
      EEPROM.put(FEEDER_RPM_ADDRESS, (uint16_t)command_x);
      break;
    case (11):
      EEPROM.put(FEEDER_ACCEL_ADDRESS, (uint16_t)command_x);
      break;
    case (12):
      EEPROM.put(FEEDER_DECEL_ADDRESS, (uint16_t)command_x);
      break;
    case (13):
      EEPROM.put(FEEDER_POLARITY_ADDRESS, (bool)command_x);
      break;
    case (20):
      EEPROM.put(SPINDLE_RPM_ADDRESS, (uint16_t)command_x);
      break;
    case (21):
      EEPROM.put(SPINDLE_ACCEL_ADDRESS, (uint16_t)command_x);
      break;
    case (22):
      EEPROM.put(SPINDLE_DECEL_ADDRESS, (uint16_t)command_x);
      break;
    case (23):
      EEPROM.put(SPINDLE_POLARITY_ADDRESS, (bool)command_x);
      break;
    case (31):
      EEPROM.put(MICROSTEPS_ADDRESS, (uint8_t)command_x);
      break;
    case (99):
      loadDefaultSettings();
      break;
    default:
      // printHelp();
      break;
    }

    break;
  }

  feederHomming();
}

void printHelp(char _command_char = ' ', uint8_t _command_num = 0)
{
  // Print help menu or ussage
  Serial.println();
}

void sendProgress()
{
  // Get current controller speed from the faster motor
  int speed = step_controller.getCurrentSpeed();
  // Print the Feeder position and speed ratio as its slower
  Serial.printf("Feeder\n\tPosition: %d,\tSpeed: %d",
                feeder.getPosition(), (speed * (FEEDER_RPM / SPINDLE_RPM)));
  Serial.println();

  // Print the Spinde position and speed as its the leading motor
  Serial.printf("Spindle\n\tTurns: %d,\tSpeed: %d\n",
                spindle.getPosition() / STEPS_PER_REV, speed);
  Serial.println();
}
