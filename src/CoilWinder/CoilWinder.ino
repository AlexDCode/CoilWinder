/*




  ---------------- TODO: ----------------
  * Create project description
  * Create overrideSpeed() function to change the speed of the winding to a percentage of the established RPM using a potentiometer.
  * Create Menu and "G-Code" Standard 2-Way
  * Create script for Slave
*/

#include <Arduino.h>
#include "CoilWinder.h"
#include "EEPROM.h"
#include "TeensyStep.h"
#include <math.h>
#include <vector>

int FEEDER_RPM;
int SPINDLE_RPM;
int FEEDER_ACCEL;
int FEEDER_DECEL;
int SPINDLE_ACCEL;
int SPINDLE_DECEL;
const int STEPS_PER_REV = MOTOR_STEPS * MICROSTEPS;
const float FEEDER_STEPS_PER_MILIMETER = STEPS_PER_REV / LEADSCREW_PITCH; // 1 revolution moves 2 mm

// Stepper Driver Objects for feeder ans spindle
Stepper feeder(FEEDER_STEP_PIN, FEEDER_DIR_PIN);
Stepper spindle(SPINDLE_STEP_PIN, SPINDLE_DIR_PIN);

// Stepper Drivers Controller
StepControl step_controller(STEP_PULSE_WIDTH, SPEED_UPDATE_PERIOD); // Moving by position

volatile bool leftLimitFlag = false;
volatile bool rightLimitFlag = false;
volatile bool homingRequired = false;

std::vector<int32_t> coil_feederSteps;
std::vector<int32_t> coil_spindleSteps;
int32_t layers;
int32_t turns;

void setup()
{
  // Start serial communication
  delay(1000);
  Serial.begin(SERIAL_BAUD_RATE);
  while (Serial.available())
  {
    return;
  }

  EEPROM.get(FEEDER_RPM_ADDRESS, FEEDER_RPM);
  EEPROM.get(SPINDLE_RPM_ADDRESS, SPINDLE_RPM);
  EEPROM.get(FEEDER_ACCEL_ADDRESS, FEEDER_ACCEL);
  EEPROM.get(FEEDER_DECEL_ADDRESS, FEEDER_DECEL);
  EEPROM.get(SPINDLE_ACCEL_ADDRESS, SPINDLE_ACCEL);
  EEPROM.get(SPINDLE_DECEL_ADDRESS, SPINDLE_DECEL);

  // Micro switch as inputs with pullup resistors
  pinMode(END_STOP_2_PIN, INPUT_PULLUP);
  pinMode(END_STOP_1_PIN, INPUT_PULLUP);

  // Stepper settings as outputs
  pinMode(FEEDER_RST_PIN, OUTPUT);
  pinMode(SPINDLE_RST_PIN, OUTPUT);
  pinMode(FEEDER_EN_PIN, OUTPUT);
  pinMode(SPINDLE_EN_PIN, OUTPUT);
  pinMode(SPINDLE_SLP_PIN, OUTPUT);
  pinMode(FEEDER_SLP_PIN, OUTPUT);

  // Other pins
  pinMode(LED_BUILTIN, OUTPUT);

  feeder
      //.setPullInSpeed(10)      // steps/s
      .setMaxSpeed(FEEDER_RPM)       // steps/s
      .setAcceleration(FEEDER_ACCEL) // steps/s^2
      .setStepPinPolarity(HIGH)      // driver expects active high pulses
      .setInverseRotation(false);    // Software will run stepper forward

  spindle
      //.setPullInSpeed(10)      // steps/s
      .setMaxSpeed(SPINDLE_RPM)       // steps/s
      .setAcceleration(SPINDLE_ACCEL) // steps/s^2
      .setStepPinPolarity(HIGH)       // driver expects active high pulses
      .setInverseRotation(false);     // Software will run stepper forward

  setMicrostepping(MICROSTEPS, FEEDER_MS1_PIN, FEEDER_MS2_PIN, FEEDER_MS3_PIN);
  setMicrostepping(MICROSTEPS, SPINDLE_MS1_PIN, SPINDLE_MS2_PIN, SPINDLE_MS3_PIN);

  // loadDefaultSettings(); // Load Default Settings at startup; Comment to avoid

  attachInterrupt(digitalPinToInterrupt(END_STOP_1_PIN), righEndStop, FALLING);
  attachInterrupt(digitalPinToInterrupt(END_STOP_2_PIN), leftEndStop, FALLING);
}

void loop()
{
  if (homingRequired)
  {
    feederHomming();
  }

  if (step_controller.isRunning())
  {
    // The controller returns the speed of the leading stepper. Calculate the
    // ratio for the lagging motor
    Serial.printf("Feeder\n\tPosition: %d,\tSpeed: %d\n", feeder.getPosition(),
                  step_controller.getCurrentSpeed() * (FEEDER_RPM / SPINDLE_RPM));

    Serial.printf("Spindle\n\tPosition: %d,\tSpeed: %d\n",
                  spindle.getPosition(), step_controller.getCurrentSpeed());
  }
  else
  {
    turnOffSteppers();
    delay(10000);
  }

  if (!step_controller.isRunning())
  {
    if (Serial.available())
      decodeSerial();
  }
}

void loadDefaultSettings()
{
  EEPROM.put(FEEDER_RPM_ADDRESS, (int)25000);
  EEPROM.put(SPINDLE_RPM_ADDRESS, (int)25000);
  EEPROM.put(FEEDER_ACCEL_ADDRESS, (int)5000);
  EEPROM.put(FEEDER_DECEL_ADDRESS, (int)3000);
  EEPROM.put(SPINDLE_ACCEL_ADDRESS, (int)5000);
  EEPROM.put(SPINDLE_DECEL_ADDRESS, (int)3000);
}

void setMicrostepping(int _Microstep, int _ms1_pin, int _ms2_pin, int _ms3_pin)
{

  /*
*   ------------ Microstepping Table ------------
!!  MS1:    MS2:    MS3:    Microstep Resolution:
    Low	    Low	    Low	    Full step
    High    Low	    Low	    Half step
    Low     High    Low	    Quarter step
    High    High    Low	    Eighth step
    High    High    High    Sixteenth step
  */
  int MicrosteppingTable[] = {0b000, 0b001, 0b010, 0b011, 0b111};

  pinMode(_ms1_pin, OUTPUT);
  pinMode(_ms2_pin, OUTPUT);
  pinMode(_ms3_pin, OUTPUT);

  int _microsteppingMask = MicrosteppingTable[int(log2(_Microstep))];

  digitalWrite(_ms1_pin, (_microsteppingMask & 0b001) >> 0);
  digitalWrite(_ms2_pin, (_microsteppingMask & 0b010) >> 1);
  digitalWrite(_ms3_pin, (_microsteppingMask & 0b100) >> 2);
}

void righEndStop()
{
  step_controller.emergencyStop();
  homingRequired = true;
}

void leftEndStop()
{
  step_controller.emergencyStop();
  homingRequired = true;
}

void feederHomming()
{
  turnOnSteppers();
  digitalWrite(SPINDLE_EN_PIN, HIGH); // Spindle Stepper Driver Disable
  digitalWrite(SPINDLE_SLP_PIN, LOW); // Spindle Stepper Driver Sleep

  feeder.setMaxSpeed(FEEDER_RPM / 4);
  feeder.setAcceleration(FEEDER_ACCEL / 4);

  bool rightLimit = !digitalRead(END_STOP_1_PIN);

  noInterrupts();
  cli();
  while (!rightLimit)
  {
    feeder.setTargetRel(-480000);
    step_controller.move(feeder);
    rightLimit = !digitalRead(END_STOP_1_PIN);
  }

  while (rightLimit)
  {
    feeder.setTargetRel(1);
    step_controller.move(feeder);
    rightLimit = !digitalRead(END_STOP_1_PIN);
  }

  feeder.setTargetRel(10);

  turnOffSteppers();
  delay(500);

  digitalWrite(FEEDER_RST_PIN, LOW); // Feeder Stepper Driver Reset
  delay(100);
  digitalWrite(FEEDER_RST_PIN, HIGH); // Feeder Stepper Driver Reset
  feeder.setPosition(feeder.getPosition());

  turnOnSteppers();
  step_controller.move(feeder);
  delay(500);

  interrupts();
  sei();

  feeder.setMaxSpeed(FEEDER_RPM);
  feeder.setAcceleration(FEEDER_ACCEL);

  homingRequired = false;
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
  digitalWrite(FEEDER_SLP_PIN, HIGH); // Feeder Stepper Driver Sleep
  digitalWrite(FEEDER_RST_PIN, HIGH); // Feeder Stepper Driver Reset

  digitalWrite(SPINDLE_EN_PIN, LOW);   // Spindle Stepper Driver Enable
  digitalWrite(SPINDLE_SLP_PIN, HIGH); // Spindle Stepper Driver Slee[]
  digitalWrite(SPINDLE_RST_PIN, HIGH); // Spindle Stepper Driver Reset

  digitalWrite(LED_BUILTIN, HIGH); // Spindle Stepper Driver Reset
}

void turnOffSteppers()
{
  digitalWrite(FEEDER_EN_PIN, HIGH); // Feeder Stepper Driver Disable
  digitalWrite(FEEDER_SLP_PIN, LOW); // Feeder Stepper Driver Un-Sleep

  digitalWrite(SPINDLE_EN_PIN, HIGH); // Spindle Stepper Driver Disable
  digitalWrite(SPINDLE_SLP_PIN, LOW); // Spindle Stepper Driver Un-Sleep

  digitalWrite(LED_BUILTIN, LOW); // Spindle Stepper Driver Reset
}

void coilCharacterization(float _coil_width, float _coil_height, float _wire_gauge_mm, float _turns, int32_t &_layers)
{
  float _turns_per_layer = _coil_width / _wire_gauge_mm;

  _layers = int32_t(_turns / _turns_per_layer);

  if (_turns_per_layer / _turns > _layers)
    _layers++;

  float _calculated_height = _layers * _wire_gauge_mm;

  if (_calculated_height > _coil_height)
    Serial.println("[ERROR]: Coil Height is too small");

  coil_feederSteps.clear();
  coil_spindleSteps.clear();
  delay(1000);

  for (int32_t i = 0; i < _layers; i++)
  {
    if (i % 2 == 0)
      coil_feederSteps.push_back(int32_t(FEEDER_STEPS_PER_MILIMETER * _coil_width));
    else
      coil_feederSteps.push_back(int32_t(-1 * _coil_width * FEEDER_STEPS_PER_MILIMETER));

    if (i == (_layers - int32_t(1)))
      coil_spindleSteps.push_back(int32_t((_turns - (_layers - 1) * _turns_per_layer) * STEPS_PER_REV));
    else
      coil_spindleSteps.push_back(int32_t(_turns_per_layer * STEPS_PER_REV));
  }
}

void buildCoil()
{
  delay(2000);

  for (int32_t i = 0; i < layers; i++)
  {
    feeder.setTargetRel(int32_t(coil_feederSteps[i]));
    spindle.setTargetRel(int32_t(coil_spindleSteps[i]));
    if (!homingRequired && !step_controller.isRunning())
    {
      // rotate_controller.rotateAsync(feeder, spindle);
      step_controller.moveAsync(feeder, spindle);
      uint prev_time = millis();
      uint dt = 500;
      while (!homingRequired && step_controller.isRunning())
      {
        if (step_controller.isRunning() && (millis() - prev_time > dt))
        {
          Serial.printf("Spindle\n\tRevolutions: %d,\tSpeed: %d\n",
                        spindle.getPosition() / STEPS_PER_REV, layers * turns);
          Serial.println();
          prev_time = millis();
        }
      }
    }
    else
      break;

    delay(250);
  }

  feeder.setTargetAbs(0);
  step_controller.move(feeder);
}

void decodeSerial()
{
  // Create a "Micro G-Code" standard to adjust settings and submit
  // Example: S01 -> split S: Settings/R:Coil, 01: FEEDER_RPM/etc.

  // Decode incoming info (Function Table/Mapping)
  int32_t S_Commands[] = {00, 01, 02, 03, 04}; // Setting Table: S00, S01, S02, etc.
  int32_t R_Commands[] = {00, 01, 02, 03, 04}; // Run Mapping: R00, R01, R02, etc.

  // Adjust settings

  // Build Coils
  float turns;
  float wire_gauge_mm;
  float coil_width;
  float coil_height;
  coilCharacterization(coil_width, coil_height, wire_gauge_mm, turns, layers);

  turnOnSteppers();
  feederHomming();
  buildCoil();
  turnOffSteppers();
}

void sendProgress()
{
  // Standarize how to send the progress and stats "G-Code"
}

void changeSettings()
{
}