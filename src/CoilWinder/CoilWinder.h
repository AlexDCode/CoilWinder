void emergencyStop();                                                                                                         // Gabriel
void loadDefaultSettings();                                                                                                   // Gabriel
void setMicrostepping(uint16_t _Microstep, uint16_t _ms1_pin, uint16_t _ms2_pin, uint16_t _ms3_pin);                          // Alex
void feederHomming();                                                                                                         // Alex
void turnOnSteppers();                                                                                                        // Gabriel
void turnOffSteppers();                                                                                                       // Gabriel
void coilCharacterization(float _coil_width, float _wire_gauge_mm, float _turns, float &_turns_per_layer, uint32_t &_layers); // Alex
void buildCoil();                                                                                                             // Alex
void decodeSerial();                                                                                                          // Gabriel
void decodeCommand(char command, int16_t value);                                                                              // Gabriel
void printHelp();                                                                                                             // Gabriel

// Define Pin
#define CLK_PIN 0
#define DT_PIN 1
#define SW_PIN 2
#define SCL_PIN 3
#define SDA_PIN 4
#define END_STOP_2_PIN 7 // End stop micro switch for secondary feeder tower
#define END_STOP_1_PIN 8 // End stop micro switch for primary feeder tower
#define FEEDER_DIR_PIN 14
#define FEEDER_STEP_PIN 15
#define FEEDER_SLP_PIN 16
#define FEEDER_RST_PIN 17
#define FEEDER_MS3_PIN 18
#define FEEDER_MS2_PIN 19
#define FEEDER_MS1_PIN 20
#define FEEDER_EN_PIN 21 //
#define STATUS_LED_PIN_R 28
#define STOP_BTN 29
#define BUZZER_PIN 30
#define SPINDLE_EN_PIN 31
#define SPINDLE_MS1_PIN 32
#define SPINDLE_SLP_PIN 34
#define SPINDLE_RST_PIN 35
#define SPINDLE_DIR_PIN 36
#define SPINDLE_STEP_PIN 37
#define SPINDLE_MS3_PIN 38
#define SPINDLE_MS2_PIN 39

// Define Stepper Motors Characteristics
#define MOTOR_STEPS 200 // Motor steps per revolution
#define STEP_PULSE_WIDTH 3
#define SPEED_UPDATE_PERIOD 3000
#define LEADSCREW_PITCH 8.0
#define FEEDER_RPM_DEFAULT 1000
#define SPINDLE_RPM_DEFAULT 1000
#define FEEDER_ACCEL_DEFAULT 5000
#define FEEDER_DECEL_DEFAULT 3000
#define FEEDER_OFFSET_DEFAULT 750
#define SPINDLE_ACCEL_DEFAULT 5000
#define SPINDLE_DECEL_DEFAULT 3000
#define FEEDER_POLARITY_DEFAULT false
#define SPINDLE_POLARITY_DEFAULT false
#define MICROSTEPS_DEFAULT 16
#define SPEED_FACTOR_DEFAULT 1
// Define Communication parameters
#define SERIAL_BAUD_RATE 115200
#define LCD_ADDRESS 0x70
#define COMMAND_SIZE 2 // 2 elements of 16 bits each

// EEPROM Variable Mapping
#define FEEDER_RPM_ADDRESS 0
#define SPINDLE_RPM_ADDRESS 32
#define FEEDER_ACCEL_ADDRESS 64
#define FEEDER_DECEL_ADDRESS 96
#define SPINDLE_ACCEL_ADDRESS 128
#define SPINDLE_DECEL_ADDRESS 160
#define FEEDER_POLARITY_ADDRESS 192
#define SPINDLE_POLARITY_ADDRESS 193
#define MICROSTEPS_ADDRESS 194
#define FEEDER_OFFSET_ADDRESS 202
#define SPEED_FACTOR_ADDRESS 234