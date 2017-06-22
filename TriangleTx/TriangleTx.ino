/*
This code:
  Identifies two robots in an environment
  Records their relative distance and bearings
  Determines which robot should move
  Instructs a robot to move to complete an equilateral triangle
  
  Note: This code is intented to run on the "Master," XBee address 0x1234
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
uint8_t idB = 0x0000;
uint8_t idC = 0x0000;

void setup() {
   // Initialize Serial output for debugging
  Serial.begin(9600);
  // Initialize I2C
  Wire.begin();
  // Initialize Robot object
  botA.init_Robot();
}

void loop() {
    // Signify that we are looking to confirm the position
    botB.pos.control = 1;
    // Detect the first unknown robot, denoting it botB
    botA.findObject(botB.pos);
    
    while (!endB){
      // Send the recorded position to a given robot and determine its ID
      success = botA.sendPosition(0x2345, botB.pos);
      // Other robot will send back a bool signifying whether it is the robot in question
      foundB = botA.receiveConfirmation();
      if (foundB == true){
        botA.flashLed(green_LED, 8);
        delay(500);
        botA.flashLed(orange_LED, 8);
        delay(500);
        botA.flashLed(green_LED, 8);
        delay(500);
        botA.flashLed(orange_LED, 8);
        delay(500);
        idB = 0x2345;
        endB = true;
        //break;
      } else {
        foundB = false;
        botA.flashLed(red_LED, 8);
        delay(500);
        botA.flashLed(orange_LED, 8);
        delay(500);
        botA.flashLed(red_LED, 8);
        delay(500);
        botA.flashLed(orange_LED, 8);
        delay(500);
        // Try the other robot
        success = botA.sendPosition(0x3456, botB.pos);
        // Other robot will send back a bool signifying whether it is the robot in question
        foundB = botA.receiveConfirmation();
        if (foundB){
          idB = 0x3456;
          endB = true;
          //break;
        } else {
          // Unable to find
        }
      }
  }
  
  // Rotate the robot to avoid detecting the other object
  botA.motor.rotateArdumotoCW(150);
  delay(100);
  botA.motor.rotateArdumotoCW(150);
  delay(100);
  botA.motor.rotateArdumotoCW(150);
  delay(100);
  botA.motor.rotateArdumotoCW(150);
  delay(100);

  // Signify that we are looking to confirm the position
  botC.pos.control = 1;
  // Detect the second robot, denoting it botC
  botA.findObject(botC.pos);

  while(!endC){
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
  
  ////// -------- TEST CODE ------------
  /*
  float inches;
  Ultrasonic ultrasonic(7);
  ultrasonic.DistanceMeasure();
  inches = ultrasonic.microsecondsToInches();
  Serial.println(inches);
  
  botA.motor.driveArdumoto(1,1,200);
  botA.motor.driveArdumoto(0,1,200);
  delay(3000);

  float bearing = 0;
  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  bearing = botA.addeg(euler.x(), 180);
  Serial.println(bearing);;
  
  botA.pos.control = 1;
  botA.pos.distance = 12;
  botA.pos.bearing1 = 104;
  botA.pos.bearing2 = 1;
  botA.pos.bearing = 5;
  
  /*
  // Control
  uint8_t payload[] = {0, botA.pos.control};
  // Send to bot 1
  botA.send(0x2345, payload);
  // Delay to prevent interference from subsequent transmission
  delay(1000);
  
  // Distance
  uint8_t payload1[] = {1, botA.pos.distance};
  // Send to bot 1
  botA.send(0x2345, payload1);
  // Delay to prevent interference from subsequent transmission
  delay(1000);
  
  // Bearing 1
  uint8_t payload2[] = {2, botA.pos.bearing1};
  // Send to bot 1
  botA.send(0x2345, payload2);
  // Delay to prevent interference from subsequent transmission
  delay(1000);
  
  // Bearing 2
  uint8_t payload3[] = {3, botA.pos.bearing2};
  // Send to bot 1
  botA.send(0x2345, payload3);
  // Delay to prevent interference from subsequent transmission
  delay(1000);
  
  botA.pos.control = 1;
  botA.pos.distance = 12;
  botA.pos.bearing1 = 104;
  botA.pos.bearing2 = 1;
  botA.pos.bearing = 5;
  
  foundB = botA.confirmPosition(botA.pos);
  
  delay(15000);
  */
  
  //// -------- END TEST CODE -------------- 
}
