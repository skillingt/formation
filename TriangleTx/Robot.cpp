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

  // Initialize Motor and IMU objects
  motor.init_Motor();
  bno = Adafruit_BNO055();
  bno.begin();
  bno.setExtCrystalUse(true);
  Serial.println("Made it past init_Robot");
}


bool Robot::findObject(Position &pos){
  // Written by John Dunn
  // Detects an object, rotates past it, then determines the middle
  // Instantiate Ultrasonic sensors
  Ultrasonic ultrasonic(7);

  // Declare local variables
  float inches;
  float distance;
  float distanceFinal;
  float heading1;
  float heading2;
  float heading3;
  float diff;
  int TICK_ROTATE1 = 50;
  int TICK_ROTATE2 = 50;
  int TICK_STOP1 = 25;
  int TICK_STOP2 = 100; 
  int SPEED = 150;
  int maxDistance = 60;
  int distanceBuffer = 3;
  bool detect = false;
  bool fineTune = false;

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  while (!detect)
    {
      motor.rotateArdumotoCW(SPEED*.75);
      delay(TICK_ROTATE2/1.5);
      motor.stopArdumoto(MOTOR_A);
      motor.stopArdumoto(MOTOR_B);
      delay(TICK_STOP2);
      ultrasonic.DistanceMeasure();
      inches = ultrasonic.microsecondsToInches();
      if(inches <= maxDistance){
        motor.stopArdumoto(MOTOR_A);
        motor.stopArdumoto(MOTOR_B);
        Serial.print("Detected! \n");
        detect = true;
        distance = inches;
        Serial.print("Distance:  ");Serial.print(distance);Serial.print("\n\n");
        euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        heading1 = addeg(euler.x(), 180);
      }     
    }
  Serial.print("Finding other side of object.....\n\n");
  while(!fineTune){
    motor.rotateArdumotoCW(SPEED*.75);
    delay(TICK_ROTATE2/2);
    motor.stopArdumoto(MOTOR_A);
    motor.stopArdumoto(MOTOR_B);
    delay(TICK_STOP2);
    ultrasonic.DistanceMeasure();
    inches = ultrasonic.microsecondsToInches();
    if(inches >= distance + distanceBuffer){
        motor.stopArdumoto(MOTOR_A);
        motor.stopArdumoto(MOTOR_B);
        delay(500);
        euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        heading2 = addeg(euler.x(), 180);
        diff = anglediff(heading2, heading1);
        fineTune = true;
    } 
  }
 
  diff = diff/2;

  Serial.print("Rotate CCW to Center.\n\n");
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  while(!(addeg(euler.x(), 180) >= (subdeg(heading2, diff)-2) && addeg(euler.x(), 180) <= (subdeg(heading2, diff)))){
    motor.rotateArdumotoCCW(SPEED*.75);
    delay(TICK_ROTATE2/2);
    motor.stopArdumoto(MOTOR_A);
    motor.stopArdumoto(MOTOR_B);
    delay(TICK_STOP2);
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    euler.x();
  }  
  motor.stopArdumoto(MOTOR_A);
  motor.stopArdumoto(MOTOR_B);

  Serial.print("Pointed at object.\n\n");
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  heading3 = addeg(euler.x(), 180);

  // Determine the distance to the observed object
  distanceFinal = ultrasonic.microsecondsToInches();
  Serial.print("Distance:  ");Serial.println(distance);

  // Assign values to the Position struct
  pos.bearing = heading3;
  pos.distance = distanceFinal;
}

// Given a position, rotate to opposite bearing, check distance
bool Robot::confirmPosition(Position &pos){
  // Connected to pin 7
  Ultrasonic ultrasonic(7);

  // Declare local variables
  float desired_bearing = 0.0;
  float tolerance_in = 3.0; // 3 inches
  float tolerance_deg = 5.0; // 5 degrees
  int time_delay = 50; // time delay in ms
  int speed = 200; // speed in range 0-255
  float bearing = 0;
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  Serial.print("Given Bearing: "); Serial.println(pos.bearing);

  // Calculate the desired heading based on the given heading
  desired_bearing = pos.bearing + 180.0;
  // Check bounds
  if (desired_bearing > 360){
    desired_bearing -= 360;
  }

  Serial.print("Desired Bearing: "); Serial.println(desired_bearing);

  // Determine current bearing, note sensor is backwards
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  bearing = addeg(euler.x(), 180);

  Serial.print("Current Bearing: "); Serial.println(bearing);
  Serial.print("Desired Bearing: "); Serial.println(desired_bearing);

  // Rotate to the desired bearing
  while(abs(bearing - desired_bearing) > tolerance_deg){
    // Bonus: Determine which was is faster to rotate
    // Rotate the motors
    motor.rotateArdumotoCW(speed);
    delay(time_delay);
    // Stop the motors
    motor.stopArdumoto(MOTOR_A);
    motor.stopArdumoto(MOTOR_B);
    // Take a measurement, updating the global variable bearing
    delay(time_delay);
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    bearing = addeg(euler.x(), 180);
    Serial.println(bearing);

    Serial.print("Current Bearing: "); Serial.println(bearing);
    Serial.print("Desired Bearing: "); Serial.println(desired_bearing);
  }

  // Get the distance to the object
  ultrasonic.DistanceMeasure();
  float inches = ultrasonic.microsecondsToInches();

  Serial.print("Inches: "); Serial.println(inches);
  Serial.print("Desired Inches: "); Serial.println(pos.distance);

  // Check if the distance is within reasonable bounds
  if (abs(inches - pos.distance) < tolerance_in){
    return true;
  } else {
    return false;
  }
}

bool Robot::sendPosition(uint16_t addr16, Position pos){
  // Wrapper for Robot::send
  // Breaks a Position struct into four seperate transmission due to XBee

  // Declare local variables
  int time_delay = 5000; // delay in ms
  uint8_t bearing1 = 0x0000;
  uint8_t bearing2 = 0x0000;

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
            flashLed(green_LED);
            return true;
          } else {
            // Remote XBee did not receive the packet
            flashLed(red_LED);
            return false;
          }
        }      
  } else if (xbee.getResponse().isError()) {
    // Packet received in error
    flashLed(red_LED);
    return false;
  }
  else {
    // Return packet not receieved
    flashLed(red_LED);
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
  uint8_t option = 0;
  uint16_t sendAddr = 0;
  uint8_t data = 0;

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
                  flashLed(green_LED);
                  xbee.getResponse().getRx16Response(rx16);
                  option = rx16.getOption();
                  data = rx16.getData(0);
                  if (data == 1){
                    flashLed(orange_LED);
                    return true;
                  } else {
                    flashLed(red_LED);
                    return false;
                  }
                  sendAddr = rx16.getRemoteAddress16();
          } else {
                  flashLed(green_LED);
                  xbee.getResponse().getRx64Response(rx64);
                  option = rx64.getOption();
                                    if (data){
                    flashLed(orange_LED);
                    return true;
                  } else {
                    flashLed(red_LED);
                    return false;
                  }
                  sendAddr = rx16.getRemoteAddress16();
          }
        } else {
          // Received a different type of packet than expected
          flashLed(red_LED);
          return false;
        }
      } else if (xbee.getResponse().isError()) {
        // Packet received in error
        flashLed(red_LED);
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
  uint8_t option = 0;
  uint16_t sendAddr = 0;
  uint8_t flag = 0;
  uint8_t* data;

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
                  flashLed(green_LED);
                  xbee.getResponse().getRx16Response(rx16);
                  option = rx16.getOption();
                  data = rx16.getData();
                  flag = data[0];
                  switch(flag){
                    case 0: 
                      pos.control = data[1];
                      if (pos.control == 1){
                        flashLed(orange_LED);
                      } else {
                        flashLed(red_LED);
                      }
                      break;
                    case 1: 
                      pos.distance = data[1];
                      if (pos.distance == 12){
                        flashLed(orange_LED);
                      } else {
                        flashLed(red_LED);
                      }
                      break;
                    case 2: 
                      pos.bearing1 = data[1];
                      if (pos.bearing1 == 104){
                        flashLed(orange_LED);
                      } else {
                        flashLed(red_LED);
                      }
                      break;
                    case 3: 
                      pos.bearing2 = data[1];
                      if (pos.bearing2 == 1){
                        flashLed(orange_LED);
                      } else {
                        flashLed(red_LED);
                      }
                      break;
                  }
                  sendAddr = rx16.getRemoteAddress16();
          } else {
                  flashLed(green_LED);
                  xbee.getResponse().getRx64Response(rx64);
                  option = rx64.getOption();
                  data = rx16.getData();
                  flag = data[0];
                  switch(flag){
                    case 0: 
                      pos.control = data[1];
                      if (pos.control == 1){
                        flashLed(orange_LED);
                      } else {
                        flashLed(red_LED);
                      }
                      break;
                    case 1: 
                      pos.distance = data[1];
                      if (pos.distance == 12){
                        flashLed(orange_LED);
                      } else {
                        flashLed(red_LED);
                      }
                      break;
                    case 2: 
                      pos.bearing1 = data[1];
                      if (pos.bearing1 == 104){
                        flashLed(orange_LED);
                      } else {
                        flashLed(red_LED);
                      }
                      break;
                    case 3: 
                      pos.bearing2 = data[1];
                      if (pos.bearing2 == 1){
                        flashLed(orange_LED);
                      } else {
                        flashLed(red_LED);
                      }
                      break;
                  }
                  sendAddr = rx16.getRemoteAddress16();
          }
          // Success
          flashLed(green_LED);
          return true;
        } else {
          // Received a different type of packet than expected
          flashLed(red_LED);
          return false;
        }
      } else if (xbee.getResponse().isError()) {
        // Packet received in error
        flashLed(red_LED);
        return false;
      } 
  }
}

void Robot::packStruct(Position &pos){
  // Turns two uint8_t bearings into a single uint16_t bearing
  pos.bearing = pos.bearing2 << 8;
  pos.bearing |= pos.bearing1;
}

void Robot::flashLed(byte LED) {
    int times = 5;
    int wait = 100;
    for (int i = 0; i < times; i++) {
      digitalWrite(LED, HIGH);
      delay(wait);
      digitalWrite(LED, LOW);
      
      if (i + 1 < times) {
        delay(wait);
      }
    }
}

// Find the difference between angles/bearings
float Robot::anglediff(float a, float b){
    float big;    //bigger number
    float small;  //smaller number
    float answer;
    if (a > b){
        big = a;
        small = b;
    }
    else{
        big = b;
        small = a;
    }
    answer = big - small;
    if (answer > 180){
        answer = 360 - answer;
    }
    return answer;
}

// Subtract angles (a - b)
float Robot::subdeg(float a, float b){
    float answer;
    answer = a - b;
    if (answer < 0){
        answer = answer + 360;
    }
    return answer;
}

// Adds two angles
float Robot::addeg(float a, float b){
    float answer;
    answer = a + b;
    if (answer > 360){
        answer = answer - 360;
    }
    return answer;
}

// Return the opposite direction
float Robot::backAngle(float a) {
  float answer;
  if (a < 180) answer = a + 180;
  else answer = a - 180;
  return answer;
}

// Add or substract beta for final direction of robot.
float Robot::betaDirSelect(float brgMOV, float brgREF, float beta) {
  float chooseDir;
  float reverseBrgMov = backAngle(brgMOV);
  float reverseBrgRef = backAngle(brgREF);
  float betaPlus, betaMinus;
  float compare1, compare2;
  if (anglediff(brgMOV, brgREF) < 60) {
    betaPlus = addeg(reverseBrgMov, beta);
    betaMinus = subdeg(reverseBrgMov, beta);
    compare1 = anglediff(reverseBrgRef, betaPlus);
    compare2 = anglediff(reverseBrgRef, betaMinus);
    if (compare1 < compare2)
      chooseDir = betaPlus;
    else
      chooseDir = betaMinus;
  }
  else if (anglediff(brgMOV, brgREF) > 60 && anglediff(brgMOV, brgREF) <= 90) {
    betaPlus = addeg(reverseBrgMov, beta);
    betaMinus = subdeg(reverseBrgMov, beta);
    compare1 = anglediff(reverseBrgRef, betaPlus);
    compare2 = anglediff(reverseBrgRef, betaMinus);
    if (compare1 < compare2)
      chooseDir = betaMinus;
    else 
      chooseDir = betaPlus;
  }
  else if (anglediff(brgMOV, brgREF) > 90) {
    betaPlus = addeg(reverseBrgMov, beta);
    betaMinus = subdeg(reverseBrgMov, beta);
    compare1 = anglediff(brgREF, betaPlus);
    compare2 = anglediff(brgREF, betaMinus);
    if (compare1 < compare2)
      chooseDir = betaPlus;
    else
      chooseDir = betaMinus;
  }
  return chooseDir;
}


void Robot::findTriangle(Position &pos, Position &pos2, Position &pos3){
  float brgX, brgY, rngX, rngY, brgREF, brgMOV, rngREF, rngMOV, brgNEW, pi;
  pi = 3.14159;
  brgX = pos.bearing;
  brgY = pos2.bearing;
  rngX = pos.distance;
  rngY = pos2.distance;

  // MAST scans cw for bots
    // MAST finds first bot, records (brgX, rngX)
    // MAST finds second bot, records (brgY, rngY)
    // (brgX, rngX) and (brgY, rngY) given for test

    //cout << "Bots and movement expressed as polar vectors (r, theta)." << endl;
    //cout << "Initial X: " << rngX << " , " << brgX << endl;
    //cout << "Initial Y: " << rngY << " , " << brgY << endl << endl;

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

  // Add case if angleDiff(brgX, brgY) ~= 60

  // Find brgNEW
  float brgNEWa = addeg(brgREF, 60);
  float brgNEWb = subdeg(brgREF, 60);
  float a = anglediff(brgMOV, brgNEWa);
  float b = anglediff(brgMOV, brgNEWb);
    // Use min(a, b) to name brgNEWa or brgNEWb brgNEW
  if (a <= b) brgNEW = brgNEWa;
  else brgNEW = brgNEWb;

  // Find brgC, which is used to find distance to move
  float brgC = anglediff(brgMOV, brgNEW);

  // Find distance to move rngC
  float rngC = sqrt(pow(rngREF, 2) + pow(rngMOV, 2) - 2 * rngREF*rngMOV*cos(brgC*pi / 180));

  // Find brgB
  float brgB = asin(rngREF*sin(brgC*pi / 180) / rngC) * 180 / pi;

  // Find turn direction for MOV
  float brgBFinal = betaDirSelect(brgMOV, brgREF, brgB);

  // Move MOV, in brgBFinal, rngC centimeters
  // Update third Position struct with movement information
  pos3.distance = rngC;
  pos3.bearing = brgBFinal;
}