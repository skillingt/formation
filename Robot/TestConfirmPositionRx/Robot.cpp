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

  // Initialize Motor and Range objects
  motor.init_Motor();
  range.init_Range();
  mag.begin();
}

bool Robot::findObject(Position &pos){
  // Written by John Dunn
  // Detects an object, rotates past it, then determines the middle

  // Instantiate IMU and Ultrasonic sensors
  Adafruit_BNO055 bno = Adafruit_BNO055();
  // Connected to pin 7
  Ultrasonic ultrasonic(7);

  // Declare local variables
  float inches;
  float distance;
  float distanceFinal;
  float heading1;
  float heading2;
  float heading3;
  float diff;
  int TICK_ROTATE1 50
  int TICK_ROTATE2 50
  int TICK_STOP1 25
  int TICK_STOP2 100 
  int SPEED 150
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
  // Declare local variables
  float desired_bearing = 0.0;
  float tolerance_in = 3.0; // 3 inches
  float tolerance_deg = 5.0; // 5 degrees
  int time_delay = 50; // time delay in ms
  int speed = 200; // speed in range 0-255
  float bearing = 0;

  // Calculate the desired heading based on the given heading
  desired_bearing = pos.bearing + 180.0;
  // Check bounds
  if (desired_bearing > 360){
    desired_bearing -= 360;
  }

  // Determine current bearing
  bearing = mag.readCompass();

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
    bearing = mag.readCompass();
    Serial.println(bearing);
  }

  // Get the distance to the object
  float inches = range.GetInches();

  // Check if the distance is within reasonable bounds
  if (abs(inches - pos.distance) < tolerance_in){
    return true;
  } else {
    return false;
  }
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

float Robot::anglediff(float a, float b)     //find the difference between angles/bearings.
{
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

float Robot::subdeg(float a, float b)        //subtract angles (a - b)
{
    float answer;
    answer = a - b;
    if (answer < 0){
        answer = answer + 360;
    }
    return answer;
}

float Robot::addeg(float a, float b)     // adds two angles
{
    float answer;
    answer = a + b;
    if (answer > 360){
        answer = answer - 360;
    }
    return answer;
}