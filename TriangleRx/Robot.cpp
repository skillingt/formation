/* 
  Robot.cpp - Class for autonomous formation robots
  Created by Taylor Skilling, April 25th, 2017.
  Released into the public domain.
*/

#include "Robot.h"

void Robot::init_Robot()
{
  // All pins should be setup as outputs:
  pinMode(green_LED, OUTPUT);
  pinMode(orange_LED, OUTPUT);
  pinMode(red_LED, OUTPUT);

  // Initialize all pins as low:
  digitalWrite(green_LED, LOW);
  digitalWrite(orange_LED, LOW);
  digitalWrite(red_LED, LOW);

  // Initialize Motor, IMU, and Ultrasonic objects
  motor.init_Motor();
  bno = Adafruit_BNO055();
  bno.begin();
  bno.setExtCrystalUse(true);
  ultrasonic.begin(7, 6);
}

void Robot::rotateToBearing(Position &pos){
  // Rotate the robot until it reaches a desired bearing
  // Declare local variables
  // Motor parameters
  uint8_t rotate_speed = 80;
  uint8_t rotate_time = 30;
  uint8_t rotate_delay = 30;
  // Measurements
  double desired_bearing = pos.bearing;
  double current_bearing = 0.0;
  // Conditions
  double tolerance_deg = 5.0; // 5 degrees

  // IMU 
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // Determine current bearing, noting sensor is "backwards"
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  current_bearing = addAngle(euler.x(), 180.0);

  // Determine which way to rotate the robot
  // Subtract the desired bearing from the current
  uint16_t direction = subAngle(desired_bearing, current_bearing);

  // Rotate to the desired bearing
  while(abs(current_bearing - desired_bearing) > tolerance_deg){
    // Rotate the robot based on the fastest direction
    if (direction  < 180){
      motor.rotateArdumotoCW(rotate_speed);
    } else {
      motor.rotateArdumotoCCW(rotate_speed);
    }
    delay(rotate_time);
    // Stop the motors
    motor.stopArdumoto(MOTOR_A);
    motor.stopArdumoto(MOTOR_B);
    // Take a measurement, updating the global bearing variable 
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    current_bearing = addAngle(euler.x(), 180.0);
    delay(rotate_delay);
  }
}

bool Robot::confirmPosition(Position &pos){
  // Given a position, rotate to 180 opposite bearing, 
  // check the distance to an object and send to the master
  
  // Declare local variables
  double tolerance_in = 5.0; // 5 inches

  // Calculate the reverse angle
  pos.bearing = backAngle(pos.bearing);

  rotateToBearing(pos);

  // Get the distance to the object
  ultrasonic.DistanceMeasure();
  long inches = ultrasonic.microsecondsToInches();

  // Check if the distance is within reasonable bounds
  if (abs(inches - pos.distance) < tolerance_in){
    return true;
  } else {
    return false;
  }
}

bool Robot::sendPosition(uint16_t addr16, Position &pos){
  // Wrapper for Robot::send
  // Breaks a Position struct into four seperate transmission due to XBee

  // Declare local variables
  int time_delay = 500; // delay in ms

  // Split bearing from a uint16_t to two uint8_ts 
  pos.bearing1 = pos.bearing & 0xff;
  pos.bearing2 = (pos.bearing >> 8);

  // Control
  uint8_t payload[] = {0, pos.control};
  // Send
  send(addr16, payload);
  // Delay to prevent interference from subsequent transmission
  delay(time_delay);
  
  // Distance
  uint8_t payload1[] = {1, pos.distance};
  // Send
  send(addr16, payload1);
  // Delay to prevent interference from subsequent transmission
  delay(time_delay);
  
  // Bearing 1
  uint8_t payload2[] = {2, pos.bearing1};
  // Send
  send(addr16, payload2);
  // Delay to prevent interference from subsequent transmission
  delay(time_delay);
  
  // Bearing 2
  uint8_t payload3[] = {3, pos.bearing2};
  // Send to
  send(addr16, payload3);
  // Delay to prevent interference from subsequent transmission
  delay(time_delay);

  return true;
}

bool Robot::send(uint16_t addr16, uint8_t* payload){
  // Create an XBee object
  XBee xbee = XBee();
  Serial.begin(9600);
  xbee.setSerial(Serial);

  // Create a Series 1 TX packet which contains payload and source address
  // Note that the first argument is the MY address of the RX Xbee
  // Note this must be run from XBee 1 as 0x3456 is XBee 2
  Tx16Request tx = Tx16Request(addr16, payload, sizeof(payload));

  // Create a TxStatusResponse object to hold the response from the receiver
  TxStatusResponse txStatus = TxStatusResponse();

  // Send the transmit request object
  xbee.send(tx);
  
  // After sending a tx request, we expect a status response
  // Turn on idle LED
  digitalWrite(orange_LED, HIGH);
  // wait up to 5 seconds for the status response
  if (xbee.readPacket(5000)) {
      // Response detected, turn off idle LED
      digitalWrite(orange_LED, LOW);
      // Expecting a response            
      if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
        xbee.getResponse().getTxStatusResponse(txStatus);
        // Get the delivery status, the fifth byte
          if (txStatus.getStatus() == SUCCESS) {
            // Success
            flashLed(green_LED, 2);
            return true;
          } else {
            // Remote XBee did not receive the packet
            flashLed(red_LED, 2);
            return false;
          }
        }      
  } else if (xbee.getResponse().isError()) {
    // Packet received in error
    flashLed(red_LED, 2);
    return false;
  }
  else {
    // Return packet not receieved
    flashLed(red_LED, 2);
    return false;
  }
}

bool Robot::receive(Position &pos){
  // Instantiate objects
  XBee xbee = XBee();
  XBeeResponse response = XBeeResponse();
  Serial.begin(9600);
  xbee.setSerial(Serial);

  // Declare local variables
  uint8_t option = 0x00;
  uint16_t sendAddr = 0x0000;
  uint8_t flag = 0x00;
  uint8_t* data = 0x00;

  // Create reusable response objects for responses we expect to handle 
  Rx16Response rx16 = Rx16Response();
  Rx64Response rx64 = Rx64Response();

  // Continuously reads packets, looking for RX16 or RX64
  // Enable idle LED for visual aid
  digitalWrite(orange_LED, HIGH);
  while(true){
      // Try and receive a packet
      xbee.readPacket();
      if (xbee.getResponse().isAvailable()) {
        // Something received, disable idle LED
        digitalWrite(orange_LED, LOW);
        if (xbee.getResponse().getApiId() == RX_16_RESPONSE || xbee.getResponse().getApiId() == RX_64_RESPONSE) {
          // Received an Rx packet
          if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
            xbee.getResponse().getRx16Response(rx16);
            option = rx16.getOption();
            data = rx16.getData();
            // Determine which piece of data was receieved
            flag = data[0];
            // Update the position structure with received data
            switch(flag){
              case 0: 
                pos.control = data[1];
                flashLed(green_LED, 1);
                break;
              case 1: 
                pos.distance = data[1];;
                flashLed(green_LED, 2);
                break;
              case 2: 
                pos.bearing1 = data[1];
                flashLed(green_LED, 3);
                break;
              case 3: 
                pos.bearing2 = data[1];
                flashLed(green_LED, 4);
                break;
            }
            sendAddr = rx16.getRemoteAddress16();
          } else {
            xbee.getResponse().getRx64Response(rx64);
            option = rx64.getOption();
            data = rx16.getData();
            flag = data[0];
            switch(flag){
              case 0: 
                pos.control = data[1];
                flashLed(green_LED, 1);
                break;
              case 1: 
                pos.distance = data[1];
                flashLed(green_LED, 2);
                break;
              case 2: 
                pos.bearing1 = data[1];
                flashLed(green_LED, 3);
                break;
              case 3: 
                pos.bearing2 = data[1];
                flashLed(green_LED, 4);
                break;
            }
            sendAddr = rx16.getRemoteAddress16();
          }
          return true;
        } else {
          // Received a different type of packet than expected
          flashLed(red_LED, 3);
          return false;
        }
      } else if (xbee.getResponse().isError()) {
        // Packet received in error
        flashLed(red_LED, 3);
        return false;
      } 
  }
}

void Robot::packStruct(Position &pos){
  // Turns two uint8_t bearings into a single uint16_t bearing
  pos.bearing = pos.bearing2 << 8;
  // Bitwise OR
  pos.bearing |= pos.bearing1;
}

void Robot::flashLed(byte pin, int times) {
  // Flash an LED given the pin and number of times to blink
  int wait = 100;
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(wait);
    digitalWrite(pin, LOW);
    if (i + 1 < times) {
      delay(wait);
    }
  }
}

uint16_t Robot::diffAngle(uint16_t a, uint16_t b){
  // Returns the difference between two angles [0, 180]
  return (abs(a - b) > 180) ? abs(abs(a - b) - 360) : abs(a - b);
}

uint16_t Robot::subAngle(uint16_t a, uint16_t b){
  // Returns the result of (a - b)
  uint16_t answer;
  answer = a - b;
  return (answer < 0) ? answer += 360 : answer;
}

uint16_t Robot::addAngle(uint16_t a, uint16_t b){
  // Returns the result of (a - b)
  uint16_t answer;
  answer = a + b;
  return (answer > 360) ? (answer - 360) : answer;
}

uint16_t Robot::backAngle(uint16_t a) {
  // Return the opposite direction
  return (a < 180) ? a + 180 : a - 180;
}