void receiveCommand(int numBytes);
void sendCommand(int16_t _command, int16_t _value = 0);
void requestData(byte _address, int16_t _command, int16_t _response[]);
// Define Communication parameters
#define SERIAL_BAUD_RATE 115200
#define COIL_WINDER_I2C_ADDRESS 0x70
#define COMMAND_SIZE 2  // 2 elements of 16 bits each
#define RESPONSE_SIZE 4 // 2 bytes of command and 2 bytes of value