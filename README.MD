# Coil Winder Final Project

<img src="assets/Images/Coil Winder Preliminary Assembly.jpg"/>

## Introduction
The goal of the project is to develop an automatic coil winding machine using commands via the serial communication.

This is just a place to store and share source code, images, and important references for documentation.

Place the source code inside the src folder, the libraries for the code in the lib folder, media files in the assets folder, and other documentation at the docs folder.

## TODO:
    * MasterControl:
      * Creating the LCD display Menu
        * Configure and Run CoilWinder
        * Progress Screen
        * Settings
      * Create a send command function that takes the parameters for the instruction and sends the data in Serial/I2C Bus
      * Use I2C for sending and receiving commands and connect to SparkFun Quiic Interface Bus
    * CoilWinder:
      * Create a send command function that takes the parameters for the instruction and sends the data in Serial/I2C Bus (also in MasterControl)
      * Use I2C for sending and receiving commands and connect to SparkFun Quiic Interface Bus (also in MasterControl)
      * Add Pinout description from hardware/Schematics/Circuit Pinouts.docx in the intro commentary @ghiraldo5
      * Verify Command Instruction Set to be compressed to 1 byte instruction, that would only make the instruction command and number to be send in a single packet and represented by an hex number @ByteCommando
      * Verify parsing function to identify and execute commands (Consider separating instruction command and number as needed) @ByteCommando
      * Create overrideSpeed() function to change the speed of the winding to a percentage of the established RPM using a potentiometer @AlexDCode
      * Create overideFeederOffset() to change the feeder offset during execution @AlexDCode
    * Hardware
      * Upgrade to Trinamic TMC2208 stepper drivers
