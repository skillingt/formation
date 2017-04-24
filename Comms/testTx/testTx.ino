/*
Initial bring-up of Xbee API communications
Tests that Xbee 1 and Xbee 2 can communicate via API mode
Run this code on Xbee 1
*/
#include <XBee.h>

// Create an XBee object
XBee xbee = XBee();

// Create a payload, noting that maximum size is 100 bytes
// This payload is simply two bytes
uint8_t payload[] = { 'H', 'I' };

// Create a Series 1 TX packet which contains payload and source address
// Note that the first argument is the MY address of the RX Xbee
// Note this must be run from XBee 1 as 0x3456 is XBee 2
Tx16Request tx = Tx16Request(0x3456, payload, sizeof(payload));

TxStatusResponse txStatus = TxStatusResponse();

void setup() {
  Serial.begin(9600);
  xbee.setSerial(Serial);
}

void loop() {
  // Send the packet
  xbee.send(tx);
  
  // After sending a tx request, we expect a status response
  // wait up to 5 seconds for the status response
  if (xbee.readPacket(5000)) {
      // got a response!

      // should be a znet tx status            	
  	if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
  	   xbee.getResponse().getTxStatusResponse(txStatus);
  		
  	 // get the delivery status, the fifth byte
         if (txStatus.getStatus() == SUCCESS) {
          	// success.  time to celebrate
           	Serial.print("Success!");
         } else {
          	// the remote XBee did not receive our packet. is it powered on?
           	Serial.print("Packet Not Received");
         }
      }      
  } else if (xbee.getResponse().isError()) {
    Serial.println("Error");
  }
  else {
    Serial.println("Undefined Error");
  }
  
   // Wait before the next transmission
   delay(3000);
}
