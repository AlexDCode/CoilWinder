void receiveCommand(int numBytes);
void sendCommand(int16_t _command, int16_t _value = 0);
void requestData(byte _address, int16_t _command, int16_t _response[]);
void decodeSerial();

// Define Communication parameters
#define SERIAL_BAUD_RATE 115200
#define COIL_WINDER_I2C_ADDRESS 0x70
#define COMMAND_SIZE 2  // 2 elements of 16 bits each
#define RESPONSE_SIZE 4 // 2 bytes of command and 2 bytes of value

#define BTN_A_PIN 2
#define BTN_B_PIN 3
#define BTN_C_PIN 4
#define CLK_PIN 8
#define DT_PIN 9
#define SW_PIN 10