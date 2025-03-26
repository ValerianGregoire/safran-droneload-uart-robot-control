#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include "dataFrameI2C.hpp"

// Baud rate for serial communication
const uint32_t baudConsole = 115200;

// Aruco markers to transmit to the esp32
const uint8_t numArucoMarkers = 2;
uint8_t currentAruco = 0;
uint8_t arucoIDs[numArucoMarkers] = {0, 1};
uint8_t simArucoX[numArucoMarkers] = {25, 75};

// I2C address of the esp32
// Define DEBUG to scan for the I2C address of the esp32
const uint8_t espAddr = 0x08;
const uint8_t correctCmd = 0x01;

// Buffer to store the data to send/receive
dataFrameI2CRecv data;

// Function declarations
void scanI2C();
void receiveEvent(int numBytes);

void setup()
{
    // Initialize serial communication
    Serial.begin(baudConsole);
    Serial.println("Serial communication initialized");

    // Initialize I2C communication
    Wire.begin();
    Wire.onReceive(receiveEvent);
    Serial.println("I2C communication initialized");
}

void loop()
{
// Scan I2C devices on the bus
#ifdef DEBUG
    Serial.println("DEBUG mode enabled");
    Serial.println("Scanning I2C devices...");
    scanI2C();
#endif

    // Send the aruco marker data to the esp32
    data.arucoID = arucoIDs[currentAruco];
    data.arucoX = simArucoX[currentAruco];

    Wire.beginTransmission(espAddr);
    Wire.write((uint8_t *)&data, sizeof(data));
    Wire.endTransmission();

    // Increment the X position of the aruco marker
    simArucoX[currentAruco] = (simArucoX[currentAruco] + 2) % 100;

    // Print the data sent to the esp32
    Serial.print("Aruco ID: ");
    Serial.println(data.arucoID);
    Serial.print("Aruco X: ");
    Serial.println(data.arucoX);

    // Wait for 0.5 second
    delay(500);
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
    // Read the received data if it is 2 bytes long
    if (Wire.read() == correctCmd)
    {
        // Change the aruco marker to be sent
        currentAruco = (currentAruco + 1) % numArucoMarkers;
        if (!currentAruco)
        {
            Serial.println("Scanned all aruco markers");
            return;
        }
    }
}
