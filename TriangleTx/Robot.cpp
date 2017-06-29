/* 
  Robot.cpp - Class for autonomous formation robots
  Created by Taylor Skilling, April 25th, 2017.
  Released into the public domain.
*/

#include "Robot.h"

void Robot::init_Robot() {
  // All pins should be setup as outputs:
  pinMode(green_LED, OUTPUT);
  pinMode(orange_LED, OUTPUT);
  pinMode(red_LED, OUTPUT);

  // Initialize all pins as low:
  digitalWrite(green_LED, LOW);
  digitalWrite(orange_LED, LOW);
  digitalWrite(red_LED, LOW);

  // Initialize Motor, IMU, and Ultrasonic Sensor objects
  motor.init_Motor();
  bno = Adafruit_BNO055();
  bno.begin();
  bno.setExtCrystalUse(true);
  ultrasonic.begin(7);
}

bool Robot::findObject(Position &pos) {
  // Detects an object, rotates past it, then determines the middle
  // Note: 180 is added to the heading due to the orientation of the sensor on the chassis

  // Measurements
  long inches;
  long distance;
  double initial_heading;
  double end_heading;
  double angle_diff;
  double desired_angle;
  // Motor parameters
  int rotate_speed = 100;
  int rotate_time = 30;
  int rotate_delay = 30;
  // Boundary conditions
  int max_distance = 30; // (in inches)
  int distance_buffer = 5;
  // Detection Logic Flags
  bool detected = false;
  bool fine_tune = false;

  // Initiate the IMU 
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  while (!detected){
    // Rotate the chassis counter clockwise incrementally
    motor.rotateArdumotoCW(rotate_speed);
    delay(rotate_time);
    motor.stopArdumoto(MOTOR_A);
    motor.stopArdumoto(MOTOR_B);
    delay(rotate_delay);
    // Measure the distance to whatever object is in front of the sensor
    ultrasonic.DistanceMeasure();
    inches = ultrasonic.microsecondsToInches();
    // If an object is within the valid distance (inches) for another robot
    if(inches <= max_distance){
      // Found another robot
      detected = true;
      motor.stopArdumoto(MOTOR_A);
      motor.stopArdumoto(MOTOR_B);
      // Record the distance and current heading
      distance = inches;
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      initial_heading = addAngle(euler.x(), 180);
    }     
  }
  // Visual confirmation of detection
  delay(1000);
  // Determine the location of the other side of the detected robot
  while(!fine_tune){
    motor.rotateArdumotoCW(rotate_speed);
    delay(rotate_time);
    motor.stopArdumoto(MOTOR_A);
    motor.stopArdumoto(MOTOR_B);
    delay(rotate_delay);
    ultrasonic.DistanceMeasure();
    inches = ultrasonic.microsecondsToInches();
    if(inches >= distance + distance_buffer){
      fine_tune = true;
      motor.stopArdumoto(MOTOR_A);
      motor.stopArdumoto(MOTOR_B);
      delay(500);
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      end_heading = addAngle(euler.x(), 180);
      // Determine the difference between the two angles
      angle_diff = diffAngle(end_heading, initial_heading);  
    } 
  }
  // Determine the midway point
  desired_angle = angle_diff / 2;
  // Visual confirmation of detection
  delay(1000);
  // Rotate to the center of the detected robot
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  while(!(addAngle(euler.x(), 180) >= (subAngle(end_heading, desired_angle)-2) && addAngle(euler.x(), 180) <= (subAngle(end_heading, desired_angle)))){
    motor.rotateArdumotoCCW(rotate_speed);
    delay(rotate_time);
    motor.stopArdumoto(MOTOR_A);
    motor.stopArdumoto(MOTOR_B);
    delay(rotate_delay);
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    euler.x();
  } 

  // Record the final heading and distance 
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  pos.bearing = addAngle(euler.x(), 180);
  ultrasonic.DistanceMeasure();
  pos.distance = ultrasonic.microsecondsToInches();
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

bool Robot::receiveConfirmation(){
  // Reads the first byte of the payload
  // A 1 indicates that the robot is confirmed
  // Anything else indicates the robot could not confirm its ID
  // Instantiate objects
  XBee xbee = XBee();
  XBeeResponse response = XBeeResponse();
  Serial.begin(9600);
  xbee.setSerial(Serial);

  // Declare local variables
  uint8_t option = 0x00;
  uint16_t sendAddr = 0x0000;
  uint8_t data_1 = 0x00;
  uint8_t data_2 = 0x00;

  // Create reusable response objects for responses we expect to handle 
  Rx16Response rx16 = Rx16Response();
  Rx64Response rx64 = Rx64Response();

  // Continuously reads packets, looking for RX16 or RX64
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
            sendAddr = rx16.getRemoteAddress16();
            option = rx16.getOption();
            data_1 = rx16.getData(0);
            data_2 = rx16.getData(1);
            if (data_1 == 0x06 && data_2 == 0x06){
              return true;
            } else if (data_1 == 0x04 && data_2 == 0x04){
              return false;
            } else {
              return false;
            }
          } else {
            xbee.getResponse().getRx64Response(rx64);
            sendAddr = rx64.getRemoteAddress64();
            option = rx64.getOption();
            data_1 = rx64.getData(0);
            data_2 = rx64.getData(1);
            if (data_1 == 0x06 && data_2 == 0x06){
              return true;
            } else if (data_1 == 0x04 && data_2 == 0x04){
              return false;
            } else {
              return false;
            }
          }
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
  // Shift upper bits left eight
  pos.bearing = pos.bearing2 << 8;
  // Bitwise OR the upper eight with the lower eight
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

// Add or substract beta for final direction of robot.
float Robot::betaDirSelect(float brgMOV, float brgREF, float beta) {
  float chooseDir;
  float reverseBrgMov = backAngle(brgMOV);
  float reverseBrgRef = backAngle(brgREF);
  float betaPlus, betaMinus;
  float compare1, compare2;
  if (diffAngle(brgMOV, brgREF) < 60) {
    betaPlus = addAngle(reverseBrgMov, beta);
    betaMinus = subAngle(reverseBrgMov, beta);
    compare1 = diffAngle(reverseBrgRef, betaPlus);
    compare2 = diffAngle(reverseBrgRef, betaMinus);
    if (compare1 < compare2)
      chooseDir = betaPlus;
    else
      chooseDir = betaMinus;
  }
  else if (diffAngle(brgMOV, brgREF) > 60 && diffAngle(brgMOV, brgREF) <= 90) {
    betaPlus = addAngle(reverseBrgMov, beta);
    betaMinus = subAngle(reverseBrgMov, beta);
    compare1 = diffAngle(reverseBrgRef, betaPlus);
    compare2 = diffAngle(reverseBrgRef, betaMinus);
    if (compare1 < compare2)
      chooseDir = betaMinus;
    else 
      chooseDir = betaPlus;
  }
  else if (diffAngle(brgMOV, brgREF) > 90) {
    betaPlus = addAngle(reverseBrgMov, beta);
    betaMinus = subAngle(reverseBrgMov, beta);
    compare1 = diffAngle(brgREF, betaPlus);
    compare2 = diffAngle(brgREF, betaMinus);
    if (compare1 < compare2)
      chooseDir = betaPlus;
    else
      chooseDir = betaMinus;
  }
  return chooseDir;
}


void Robot::findTriangle(Position &pos, Position &pos2, Position &pos3){
  // Determines the bearing and distance for a robot to drive to complete 
  // an equilateral triangle given two robots' positions

  // Declare variables
  float brgX, brgY, rngX, rngY, brgREF, brgMOV, rngREF, rngMOV, brgNEW, pi;
  pi = 3.14159;
  brgX = pos.bearing;
  rngX = pos.distance;
  brgY = pos2.bearing;
  rngY = pos2.distance;

  // MAST scans cw for bots
  // MAST finds first bot, records (brgX, rngX)
  // MAST finds second bot, records (brgY, rngY)

  // Choose REF, MOV
  //min(rngX, rngY) to name X or Y REF (whichever is closer) and the other MOV
  if (rngX <= rngY) {
    rngREF = rngX;
    brgREF = brgX;
    rngMOV = rngY;
    brgMOV = brgY;
    // Pos2 is moving
    pos3.control = 2;
  }
  else {
    rngREF = rngY;
    brgREF = brgY;
    rngMOV = rngX;
    brgMOV = brgX;
    // Pos is moving
    pos3.control = 1;
  }

  // TODO: Add case if diffAngle(brgX, brgY) ~= 60

  // Find brgNEW
  float brgNEWa = addAngle(brgREF, 60);
  float brgNEWb = subAngle(brgREF, 60);
  float a = diffAngle(brgMOV, brgNEWa);
  float b = diffAngle(brgMOV, brgNEWb);
  // Use min(a, b) to name brgNEWa or brgNEWb brgNEW
  if (a <= b) brgNEW = brgNEWa;
  else brgNEW = brgNEWb;

  // Find brgC, which is used to find distance to move
  // This is the angle C in the diagram
  float brgC = diffAngle(brgMOV, brgNEW);

  // Find distance to move rngC
  // Use the Law of Cosines to determine the distance needed to move MOV
  // The distance between the MASTER and the point to move to is known
  // as we are creating an equilateral triangle and one side length is known (REF)
  float rngC = sqrt(pow(rngREF, 2) + pow(rngMOV, 2) - 2 * rngREF*rngMOV*cos(brgC*pi / 180));

  // Find brgB
  float brgB = asin(rngREF*sin(brgC*pi / 180) / rngC) * 180 / pi;

  // Better: Use Law of Cosines again knowing rngMOV and rngRef (and rngC) and brgC (angleC)


  // Check with green light

  // Find turn direction for MOV
  float brgBFinal = betaDirSelect(brgMOV, brgREF, brgB);

  // Move MOV, in brgBFinal, rngC centimeters
  // Update third Position struct with movement information
  pos3.distance = rngC;
  pos3.bearing = brgBFinal;
}