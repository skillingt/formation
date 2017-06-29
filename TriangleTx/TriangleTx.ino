/*
This code:
  Identifies two robots in an environment
  Records their relative distance and bearings
  Determines which robot should move
  Instructs a robot to move to complete an equilateral triangle
  
  Note: This code is intented to run on the "Master," XBee address 0x1234
*/

#include <avr/sleep.h>
#include <math.h>
#include <Wire.h>
#include <XBee.h>
#include <EnableInterrupt.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Robot.h"

// Instantiate robot objects
Robot botA;
Robot botB;
Robot botC;

// Record the XBee addresses of the two other robots
uint16_t botBaddr = 0x2345;
uint16_t botCaddr = 0x3456;

// Define variables
bool foundB = false;
bool endB = false;
bool foundC = false;
bool endC = false;
bool success = false;
uint16_t idB = 0x0000;
uint16_t idC = 0x0000;

void setup() {
   // Initialize Serial output for debugging
  Serial.begin(9600);
  // Initialize I2C
  Wire.begin();
  // Initialize Robot object
  botA.init_Robot();
  // Delay in order to physically rotate the robots, calibrating IMU
  delay(6000);
}

void loop() {    
  while (!endB){
    // LED Indication
    botA.flashLed(green_LED, 3);
    // Signify that we are looking to confirm the position
    botB.pos.control = 1;
    // Detect the first unknown robot, denoting it botB
    botA.findObject(botB.pos);
    // Send the recorded position to a given robot and determine its ID
    success = botA.sendPosition(0x2345, botB.pos);
    // Other robot will send back a bool signifying whether it is the robot in question
    foundB = botA.receiveConfirmation();
    if (foundB == true){
      idB = 0x2345;
      endB = true;
    } else {
       // Rotate the robot to avoid detecting the same object twice
      botA.motor.rotateArdumotoCW(120);
      delay(175);
      botA.motor.stopArdumoto(MOTOR_A);
      botA.motor.stopArdumoto(MOTOR_B);
      // Try the other robot
      success = botA.sendPosition(0x3456, botB.pos);
      // Other robot will send back a bool signifying whether it is the robot in question
      foundB = botA.receiveConfirmation();
      if (foundB){
        idB = 0x3456;
        endB = true;
      }
    }
  }
  
  while(!endC){
    // LED Indication
    botA.flashLed(red_LED, 3);
    
    // Rotate the robot to avoid detecting the same object twice
    botA.motor.rotateArdumotoCW(120);
    delay(175);
    botA.motor.stopArdumoto(MOTOR_A);
    botA.motor.stopArdumoto(MOTOR_B);

    // Signify that we are looking to confirm the position
    botC.pos.control = 1;
    // Detect the second robot, denoting it botC
    botA.findObject(botC.pos);
    // Send the recorded position to a given robot and determine its ID
    // If the previous robot was ID 0x2345, try the other robot
    if (idB == 0x2345){
      success = botA.sendPosition(0x3456, botC.pos);
      // Other robot will send back a bool signifying whether it is the robot in question
      foundC = botA.receiveConfirmation();
      if (foundC){
        idC = 0x3456;
        endC = true;
        break;
      }
    } else {
      success = botA.sendPosition(0x2345, botC.pos);
      // Other robot will send back a bool signifying whether it is the robot in question
      foundC = botA.receiveConfirmation();
      if (foundC){
        idC = 0x2345;
        endC = true;
      }
    }
  }
  
  // Run the algorithm given that we know two robot coordinates
  botA.findTriangle(botB.pos, botC.pos, botA.pos);
  // Determine the moving robot from the control bit
  uint16_t moveAddr = 0x000;
  if (botA.pos.control == 1){
    // botB is moving
    moveAddr = idB;
  } else if (botA.pos.control == 2){
    // botC is moving
    moveAddr = idC;
  } else {
    // Undefined behavior
  }    
  
  // Signify that we are looking for the robot to move
  botA.pos.control = 0;
  // Send the determined necessary bearing and heading to the desired robot
  success = botA.sendPosition(moveAddr, botA.pos);
  
  // Sleep
  cli();
  sleep_enable();
  sleep_cpu();
}
