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
uint8_t confirmation[2];

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
    
    delay(3000);
    
    // Construct a message to send back to the Master
    if (success == true){
      botA.flashLed(green_LED, 10);
      uint8_t confirmation[] = { 1, 0 };
    } else if (success == false){
      botA.flashLed(red_LED, 10);
      uint8_t confirmation[] = { 0, 0 };
    } else {
      botA.flashLed(red_LED, 3);
      botA.flashLed(orange_LED, 3);
      botA.flashLed(green_LED, 3);
    }
    
    delay(3000);
    
    // Send the payload
    botA.send(botBaddr, confirmation);
    
    botA.flashLed(green_LED, 5);
    delay(500);
    botA.flashLed(red_LED, 5);
    delay(500);
    botA.flashLed(green_LED, 5);
    
    delay(3000);
  } else if (botA.pos.control == 0){
    // Drive to given position to form a triangle
    // Rotate
    botA.rotateToBearing(botA.pos);
    // Drive
    // Convert inches to centimeters
    float centimeters = botA.pos.distance * 2.54;
    uint8_t speed = 200;
    botA.motor.driveDistanceStr8(speed, centimeters);
  } else {
    // Control bit not identified
  }
  
  ////// -------- TEST CODE ------------
  /*
  float inches;
  Ultrasonic ultrasonic(7, 6);
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
  botA.packStruct(botA.pos);
  botA.pos.bearing = 56;
  bool foundB = botA.confirmPosition(botA.pos);
  
  delay(15000);
  */
  //// -------- END TEST CODE -------------- 
}
