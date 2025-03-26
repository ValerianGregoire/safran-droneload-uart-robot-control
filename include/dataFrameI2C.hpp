#include <Arduino.h>

/*
The drone must place itself above the robot's aruco and start tracking one static
aruco marker. The robot will move to the left or to the right depending on the
position of the aruco marker in the camera frame. The robot will stop moving when
the aruco marker is aligned with the robot.
*/

// Dataframe to be sent from the RasPi to the esp32 via I2C
struct dataFrameI2CRecv {
    uint8_t arucoID; // ID of the aruco marker currently detected by the camera
    int arucoX;  // X position of the aruco marker in the camera frame
};