/*


*/

#include "TeensyStep.h"
#include <Arduino.h>

void righEndStop();
void leftEndStop();

// Pin definitions
#define RX_PIN 0         // Pin for receiving UART data
#define TX_PIN 1         // Pin for transmitting UART data
#define END_STOP_2_PIN 7 // End stop micro switch for secondary feeder tower
#define END_STOP_1_PIN 8 // End stop micro switch for primary feeder tower
#define FEEDER_EN_PIN 21 //
#define FEEDER_MS1_PIN 20
#define FEEDER_MS2_PIN 19
#define FEEDER_MS3_PIN 18
#define FEEDER_RST_PIN 17
#define FEEDER_SLP_PIN 16
#define FEEDER_STEP_PIN 15
#define FEEDER_DIR_PIN 14
#define SPINDLE_EN_PIN 31
#define SPINDLE_MS1_PIN 32
#define SPINDLE_MS2_PIN 39
#define SPINDLE_MS3_PIN 38
#define SPINDLE_RST_PIN 35
#define SPINDLE_SLP_PIN 34
#define SPINDLE_STEP_PIN 37
#define SPINDLE_DIR_PIN 36

// Define global parameters
#define SERIAL_BAUD_RATE 115200

#define MOTOR_STEPS                                                            \
  200 // Motor steps per revolution. Most steppers are 200 steps or 1.8
      // degrees/step
#define MICROSTEPS 16
#define FEEDER_RPM 5
#define SPINDLE_RPM 5
#define FEEDER_ACCEL 60
#define FEEDER_DECEL 40
#define SPINDLE_ACCEL 60
#define SPINDLE_DECEL 40
#define STEPS_PER_REV MICROSTEPS *MOTOR_STEPS

// Stepper Driver Objects for feeder ans spindle
Stepper feeder(FEEDER_STEP_PIN, FEEDER_DIR_PIN);
Stepper spindle(SPINDLE_STEP_PIN, SPINDLE_DIR_PIN);

// Stepper Drivers Controller
StepControl controller;

volatile byte leftLimitFlag = 0;
volatile byte rightLimitFlag = 0;

void setup() {
  // Start serial communication
  Serial.begin(SERIAL_BAUD_RATE);

  // Micro switch as inputs with pullup resistors
  pinMode(END_STOP_2_PIN, INPUT_PULLUP);
  pinMode(END_STOP_1_PIN, INPUT_PULLUP);

  feeder
      //.setPullInSpeed(10)      // steps/s
      .setMaxSpeed(FEEDER_RPM)       // steps/s
      .setAcceleration(FEEDER_ACCEL) // steps/s^2
      .setStepPinPolarity(HIGH);     // driver expects active high pulses

  spindle
      //.setPullInSpeed(10)      // steps/s
      .setMaxSpeed(SPINDLE_RPM)       // steps/s
      .setAcceleration(SPINDLE_ACCEL) // steps/s^2
      .setStepPinPolarity(HIGH);      // driver expects active high pulses

  /*
    Reset Pin is an active low input, when pulled LOW all STEP inputs are
    ignored, until you pull it HIGH. It also resets the driver by setting the
    internal translator to a predefined Home state. Home state is the initial
    position from where the motor starts and depends on the microstep
    resolution.
  */
  digitalWrite(FEEDER_RST_PIN, HIGH);  // Feeder Stepper Driver Reset
  digitalWrite(SPINDLE_RST_PIN, HIGH); // Spindle Stepper Driver Reset

  digitalWrite(FEEDER_EN_PIN, LOW);  // Feeder Stepper Driver Enable
  digitalWrite(SPINDLE_EN_PIN, LOW); // Feeder Stepper Driver Enable

  attachInterrupt(digitalPinToInterrupt(END_STOP_1_PIN), righEndStop, FALLING);
  attachInterrupt(digitalPinToInterrupt(END_STOP_2_PIN), leftEndStop, FALLING);
}

void loop() {

  if (leftLimitFlag) {
    // feeder.rotate(1);
    if (digitalRead(!END_STOP_2_PIN)) {
      leftLimitFlag = 0;

      // Define Home position
      digitalWrite(FEEDER_RST_PIN, LOW);
      delay(100);
      digitalWrite(FEEDER_RST_PIN, HIGH);
    }
  } else if (rightLimitFlag) {
    // feeder.rotate(-1);
    if (digitalRead(!END_STOP_1_PIN))
      rightLimitFlag = 0;
  }

  unsigned int wait_time = 0;
  if (wait_time) {
    Serial.print("\tFeeder Step = ");
    Serial.println();

    Serial.print("\tFeeder RPM = ");
    Serial.println(feeder.getPosition());

    Serial.print("\tSpindle Step = ");
    Serial.println(spindle.getPosition());

    Serial.print("\tSpindle RPM = ");
    Serial.println();

    Serial.print("\tTime = ");
    Serial.println(wait_time);

    Serial.println();
  } else {
    // feeder.disable();
    // spindle.disable();
    Serial.println("END");
    delay(3600000);
  }
}

void righEndStop() {
  // controller.stopAsync(); // initiate stopping procedure
  controller.emergencyStop();
  leftLimitFlag = 1;
}
void leftEndStop() {
  // controller.stopAsync(); // initiate stopping procedure
  controller.emergencyStop();
  rightLimitFlag = 1;
}