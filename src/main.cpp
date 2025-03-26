#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include "dataFrameI2C.hpp"

#define DEBUG

/*
The esp32 receives information on the current aruco marker detected by the camera
through I2C and processes it to send the corresponding command to the robot on
the ground in UART.

Valérian Grégoire--Bégranger
*/

// Baud rate for serial communication
const uint32_t baudConsole = 115200;
const uint32_t baudRobot = 57600;

// Aruco markers to align the robot with
const uint8_t numArucoMarkers = 2;
uint8_t arucoIDs[numArucoMarkers] = {0, 1};
bool arucoDetected[numArucoMarkers] = {false, false};
bool arucoAligned[numArucoMarkers] = {false, false};

// Commands to be sent to the robot
// The commands are initialized in the setup function
const uint8_t robotSpeed = 100;
const uint8_t stopBound = 2;
char stopCmd[10];
char rightCmd[10];
char leftCmd[10];

// I2C address of the RasPi
// Define DEBUG to scan for the I2C address of the RasPi
const uint8_t espAddr = 0x08;
const uint8_t correctCmd = 0x01;

// Buffer to store the received data
dataFrameI2CRecv dataBuffer;
dataFrameI2CRecv data;
char uartRecv[4];

// Function declarations
void scanI2C();
void receiveEvent(int numBytes);

void setup()
{
    // Initialize serial communication
    Serial.begin(baudConsole);
    Serial2.begin(baudRobot);
    Serial.println("Serial communication initialized");

    // Initialize I2C communication
    Wire.begin();
    Wire.onReceive(receiveEvent);
    Serial.println("I2C communication initialized");

    // Initialize the stop, right, and left commands
    sprintf(stopCmd, "S %d", robotSpeed);
    sprintf(rightCmd, "D %d", robotSpeed);
    sprintf(leftCmd, "G %d", robotSpeed);
    Serial.println("UART commands initialized");
}

void loop()
{
// Scan I2C devices on the bus
#ifdef DEBUG
    Serial.println("DEBUG mode enabled");
    Serial.println("Scanning I2C devices...");
    scanI2C();
#endif

    // Check if an aruco marker is detected
    for (uint8_t i = 0; i < numArucoMarkers; i++)
    {
        if (dataBuffer.arucoID == arucoIDs[i])
        {
            arucoDetected[i] = true;
            memcpy(&data, &dataBuffer, sizeof(dataBuffer));
            break;
        }
    }

    // Check if the detected aruco marker is aligned with the robot
    for (uint8_t i = 0; i < numArucoMarkers; i++)
    {
        if (arucoDetected[i])
        {
            // Stop the robot and wait for a OUI or NON answer
            if (abs(data.arucoX) <= stopBound)
            {
                Serial.println("Stop the robot");
                Serial2.println(stopCmd);

                // Reset the uartRecv buffer
                Serial.println("Resetting the uartRecv buffer");
                for (uint8_t i = 0; i < 4; i++)
                {
                    uartRecv[i] = 0;
                }

                // Wait for the RasPi to send a response
                Serial.println("Waiting for a response from the RasPi");
                
                // Timeout after 1 second
                uint32_t timeoutCounter = millis();
                bool timeout = false;
                while (!Serial2.available() && !timeout)
                {
                    timeout = millis() - timeoutCounter < 1000;
                }
                
                if (timeout)
                {
                    Serial.println("Timeout reached");
                    continue;
                }
                
                // Read the response from the RasPi
                uint8_t i = 0;
                while (i < 3)
                {
                    uartRecv[i] = Serial2.read();
                    i++;
                }
                uartRecv[3] = '\0';
                
                // Compare the response with the expected command
                Serial.print("Received response : ");
                Serial.println(uartRecv);
                if (strcmp(uartRecv, "OUI") == 0)
                {
                    Serial.print("arucoAligned set for aruco ");
                    Serial.println(data.arucoID);
                    arucoDetected[i] = false;
                    arucoAligned[i] = true;

                    // Send a validation message to the RasPi
                    Wire.beginTransmission(espAddr);
                    Wire.write(correctCmd);
                    Wire.endTransmission();
                }
            }
            // Move the robot to the left
            else if (data.arucoX > stopBound)
            {
                Serial.println("Move to the left");
                Serial2.println(leftCmd);
            }
            // Move the robot to the right
            else if (data.arucoX < -stopBound)
            {
                Serial.println("Move to the right");
                Serial2.println(rightCmd);
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
