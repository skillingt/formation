/*
This code:
  Identifies two robots in an environment
  Records their relative distance and bearings
  Determines which robot should move
  Instructs a robot to move to complete an equilateral triangle
  
  Note: This code is intented to run on either the reference or moving robot with XBee addresses 0x2345 or 0x3456
*/

#include <math.h>
#include <Wire.h>
#include <XBee.h>
#include <EnableInterrupt.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Robot.h"

// Instantiate robot objects
Robot botA; // This robot
Robot botB; // Master robot

// Record the XBee addresses of the master robot
uint16_t botBaddr = 0x1234;

// Declare variables
bool success;
uint8_t payload[] = {0x00, 0x00};

void setup() {
   // Initialize Serial output for debugging
  Serial.begin(9600);
  // Initialize I2C
  Wire.begin();
  // Initialize Robot object
  botA.init_Robot();
}

void loop() {
  // Wait to receive data in order to confirm position
  // Receive all 4 pieces of position data
  botA.receive(botA.pos);
  botA.receive(botA.pos);
  botA.receive(botA.pos);
  botA.receive(botA.pos);
  
  // Construct a valid bearing from the received Position struct
  botA.packStruct(botA.pos);
  // Determine if we're the identified robot
  if (botA.pos.control == 1){   
    // Confirm position
    success = botA.confirmPosition(botA.pos);

    // Construct a message to send back to the Master
    if (success == true){
      botA.flashLed(green_LED, 5);
      payload[0] = 0x06;
      payload[1] = 0x06;
    } else if (success == false){
      botA.flashLed(red_LED, 5);
      payload[0] = 0x04;
      payload[1] = 0x04;
    } else {
      botA.flashLed(red_LED, 3);
      botA.flashLed(orange_LED, 3);
      botA.flashLed(green_LED, 3);
    }
    
    // Send the payload
    botA.send(botBaddr, payload);
  } else if (botA.pos.control == 0){
    // Drive to given position to form a triangle
    // Rotate
    botA.rotateToBearing(botA.pos);
    // Drive
    // Convert inches to centimeters
    double centimeters = botA.pos.distance * 2.54;
    uint8_t travel_speed = 100;
    botA.motor.driveDistance(travel_speed, centimeters);
  } else {
    // Control bit not identified
  }
}
