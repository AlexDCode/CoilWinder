void receiveCommand();
void sendCommand(char _command, int16_t _value = 0);
void requestData(byte _address, char _command, int16_t _response[]);

// Define Communication parameters
#define SERIAL_BAUD_RATE 115200
#define COIL_WINDER_I2C_ADDRESS 0x07
#define COMMAND_SIZE (8 + 16) / 8 // 8 bits for command and 16 for response. Divide by 8 to get bytes