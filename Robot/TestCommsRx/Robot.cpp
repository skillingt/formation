/* 
  Robot.cpp - Class for autonomous formation robots
  Created by Taylor Skilling, April 25th, 2017.
  Released into the public domain.
*/

#include "Robot.h"

Robot::Robot()
{
  ID = ID;
  // All pins should be setup as outputs:
  pinMode(green_LED, OUTPUT);
  pinMode(orange_LED, OUTPUT);
  pinMode(red_LED, OUTPUT);

  // Initialize all pins as low:
  digitalWrite(green_LED, LOW);
  digitalWrite(orange_LED, LOW);
  digitalWrite(red_LED, LOW);
}

/* John to implement
Position Robot::findObject(){

}
*/

/* @TODO after confirming send and receive
bool confirmPosition(uint16_t addr16, Position current_position){

}*/

bool Robot::send(uint16_t addr16, uint8_t* payload){
  // Create an XBee object
  XBee xbee = XBee();
  Serial.begin(9600);
  xbee.setSerial(Serial);

  // Create a Series 1 TX packet which contains payload and source address
  // Note that the first argument is the MY address of the RX Xbee
  // Note this must be run from XBee 1 as 0x3456 is XBee 2
  Tx16Request tx = Tx16Request(0x2345, payload, sizeof(payload));

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
  uint8_t data = 0;
  uint16_t sendAddr = 0;

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
                  pos.control = rx16.getData(0);
                  pos.distance = rx16.getData(1);
                  pos.bearing = rx16.getData(2);
                  sendAddr = rx16.getRemoteAddress16();
          } else {
                  flashLed(green_LED);
                  xbee.getResponse().getRx64Response(rx64);
                  option = rx64.getOption();
                  pos.control = rx16.getData(0);
                  pos.distance = rx16.getData(1);
                  pos.bearing = rx16.getData(2);
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