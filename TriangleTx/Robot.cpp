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
  int rotate_speed = 80;
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

uint16_t Robot::lawOfCosines(uint16_t a, uint16_t b, uint16_t angle){
  // Return the side length c given sides a, b and the opposite angle, angle
  return sqrt(pow(a, 2) + pow(b, 2) - 2 * a * b * cos(angle * (PI / 180)));
}

uint16_t Robot::getFinalBearing(uint16_t mov_brng, uint16_t ref_brng, uint16_t rel_angle){
  // Determine the final bearing in which to turn to complete the triangle

  // Declare variables
  uint16_t final_brng = 0x0000;
  uint16_t rel_angle_plus, rel_angle_minus, diff_a, diff_b = 0x0000;

  // Determine the opposite angle of the reference and moving robot bearings
  uint16_t mov_brng_rev = backAngle(mov_brng);
  uint16_t ref_brng_rev = backAngle(ref_brng);

  // Based on the geometry, determine which bearing to rotate to relative to North
  if (diffAngle(mov_brng, ref_brng) <= 60){
    rel_angle_plus = addAngle(mov_brng_rev, rel_angle);
    rel_angle_minus = subAngle(mov_brng_rev, rel_angle);
    diff_a = diffAngle(ref_brng_rev, rel_angle_plus);
    diff_b = diffAngle(ref_brng_rev, rel_angle_minus);
    if (diff_a < diff_b){
      final_brng = rel_angle_plus;
    } else {
      final_brng = rel_angle_minus;
    }
  } else if (diffAngle(mov_brng, ref_brng) > 60 && diffAngle(mov_brng, ref_brng) <= 90){
    rel_angle_plus = addAngle(mov_brng_rev, rel_angle);
    rel_angle_minus = subAngle(mov_brng_rev, rel_angle);
    diff_a = diffAngle(ref_brng_rev, rel_angle_plus);
    diff_b = diffAngle(ref_brng_rev, rel_angle_minus);
    if (diff_a < diff_b){
      final_brng = rel_angle_minus;
    } else {
      final_brng = rel_angle_plus;
    }
  } else if (diffAngle(mov_brng, ref_brng) > 90){
    rel_angle_plus = addAngle(mov_brng_rev, rel_angle);
    rel_angle_minus = subAngle(mov_brng_rev, rel_angle);
    diff_a = diffAngle(ref_brng, rel_angle_plus);
    diff_b = diffAngle(ref_brng, rel_angle_minus);
    if (diff_a < diff_b){
      final_brng = rel_angle_plus;
    } else {
      final_brng = rel_angle_minus;
    }
  }
  return final_brng;
}

void Robot::findTriangle(Position &botB, Position &botC, Position &botA){
  // Determines the bearing and distance for a robot to drive to complete 
  // an equilateral triangle given two robots' positions

  // botB is the first detected robot, botC is the second, 
  // and botA is the struct filled with new coordinates to move to
  // This matches the nomenclature used in TriangleTx

  // Declare variables
  uint8_t ref_dist, mov_dist = 0x00;
  uint16_t ref_brng, mov_brng = 0x0000;

  // Determine which robot will move based on which one is closer to MASTER
  // Farther robot moves and is denoted MOV, closer stays and is denoted REF
  if (botB.distance <= botC.distance) {
    ref_dist = botB.distance;
    ref_brng = botB.bearing;
    mov_dist = botC.distance;
    mov_brng = botC.bearing;
    // botC is moving
    botA.control = 0x02;
  } else {
    ref_dist = botC.distance;
    ref_brng = botC.bearing;
    mov_dist = botB.distance;
    mov_brng = botB.bearing;
    // botB is moving
    botA.control = 0x01;
  }

  // Determine the bearing (relative to North) of the third vertex
  uint16_t abs_brng, vertex_a, vertex_b, diff_a, diff_b = 0x0000;
  // Third vertex must be 60 degrees from reference in an equilateral triangle
  vertex_a = addAngle(ref_brng, 60);
  vertex_b = subAngle(ref_brng, 60);
  // Determine which vertex is closer to the robot who will move
  diff_a = diffAngle(mov_brng, vertex_a);
  diff_b = diffAngle(mov_brng, vertex_b);
  // Choose the vertex which has a smaller angle relative to the moving robot's bearing
  (diff_a <= diff_b) ? abs_brng = vertex_a : abs_brng = vertex_b;

  // Determine the distance and bearing needed by MOV to complete the triangle
  // Calculate the angle between the new vertex's bearing and the bearing of MOV
  uint16_t int_angle = diffAngle(mov_brng, abs_brng);

  // Calculate the distance MOV has to travel to reach the final vertex
  // The distance between the MASTER and the vertex is known
  // as we are creating an equilateral triangle
  botA.distance = lawOfCosines(ref_dist, mov_dist, int_angle);

  // Calculate the angle to turn relative to mov_brng, converting to radians and back
  uint16_t rel_angle = asin(ref_dist * sin(int_angle * PI / 180) / botA.distance) * 180 / PI;
  // Could also use Law of Cosines knowing ref_dist, mov_dist, and botA.distance

  // Find the bearing relative to North, as there are two solutions with the relative angle
  botA.bearing = getFinalBearing(mov_brng, ref_brng, rel_angle);
}