#include <Arduino.h>
#include <Wire.h>
#include "dataFrameI2C.hpp"

#define DEBUG

/*
The esp32 receives information on the current aruco marker detected by the camera
and processes it to send the corresponding command to the robot on the ground.

Valérian Grégoire--Bégranger
*/

// Baud rate for serial communication
const uint8_t baudRobot = 57600;

// Aruco markers to align the robot with
const uint8_t numArucoMarkers = 2;
uint8_t arucoIDs[numArucoMarkers] = {0, 1};
bool arucoDetected[numArucoMarkers] = {false, false};
bool arucoAligned[numArucoMarkers] = {false, false};

// Commands to be sent to the robot
enum class command : char
{
    STOP = 'S',
    LEFT = 'G',
    RIGHT = 'D'
};
command cmd;

// I2C address of the RasPi
const uint8_t raspiAddr = 0x08;

// Buffer to store the received data
dataFrameI2CRecv dataBuffer;
dataFrameI2CRecv data;

void setup()
{
    // Initialize serial communication
    Serial2.begin(baudRobot);

    // Initialize I2C communication
    Wire.begin();
    Wire.onReceive(receiveEvent);
}

void loop()
{
// Scan I2C devices on the bus
#ifdef DEBUG
    scanI2C();
#endif

    // Check if an aruco marker is detected
    for (uint8_t i = 0; i < numArucoMarkers; i++)
    {
        if (dataBuffer.arucoID == arucoIDs[i])
        {
            arucoDetected[i] = true;
            memccpy(&data, &dataBuffer, sizeof(dataBuffer), sizeof(dataBuffer));
            break;
        }
    }

    // Check if the detected aruco marker is aligned with the robot
    for (uint8_t i = 0; i < numArucoMarkers; i++)
    {
        if (arucoDetected[i])
        {
            if (abs(data.arucoX) <= 5)
            {
                arucoAligned[i] = true;
                cmd = command::STOP;
                Serial2.write(static_cast<char>(cmd));
            }
            // Move the robot to the left
            else if (data.arucoX > 5)
            {
                arucoAligned[i] = false;
                cmd = command::LEFT;
                Serial2.write(static_cast<char>(cmd));
            }
            // Move the robot to the right
            else if (data.arucoX < -5)
            {
                arucoAligned[i] = false;
                cmd = command::RIGHT;
                Serial2.write(static_cast<char>(cmd));
            }

        }
    }
}

// Function to scan for I2C devices
// Code taken from https://randomnerdtutorials.com/esp32-i2c-communication-arduino-ide/
#ifdef DEBUG
void scanI2C()
{
    byte error, address;
    int nDevices;
    Serial.println("Scanning...");
    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.println(address, HEX);
            nDevices++;
        }
        else if (error == 4)
        {
            Serial.print("Unknow error at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
    {
        Serial.println("No I2C devices found\n");
    }
    else
    {
        Serial.println("done\n");
    }
}
#endif

// Function to receive data from the RasPi
void receiveEvent(int numBytes)
{
    // Create a dataFrameI2CRecv object to store the received data
    dataFrameI2CRecv data;

    // Read the received data if it is 2 bytes long
    if (numBytes == 2)
    {
        data.arucoID = Wire.read();
        data.arucoX = Wire.read();
    }

    // Copy the received data to the dataBuffer
    memccpy(&dataBuffer, &data, sizeof(data), sizeof(data));
}