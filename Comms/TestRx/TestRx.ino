/*
Initial bring-up of Xbee API communications
Tests that Xbee 1 and Xbee 2 can communicate via API mode
Run this code on Xbee 2
*/
#include <XBee.h>

// Instantiate objects
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();

// Create reusable response objects for responses we expect to handle 
Rx16Response rx16 = Rx16Response();
Rx64Response rx64 = Rx64Response();

uint8_t option = 0;
uint8_t data = 0;
uint16_t sendAddr = 0;

void setup() {
  // Start serial
  Serial.begin(9600);
  xbee.setSerial(Serial);
}

// Cntinuously reads packets, looking for RX16 or RX64
void loop() {
    
    xbee.readPacket();
    
    if (xbee.getResponse().isAvailable()) {
      // got something
      
      if (xbee.getResponse().getApiId() == RX_16_RESPONSE || xbee.getResponse().getApiId() == RX_64_RESPONSE) {
        // got a rx packet
        
        if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
                xbee.getResponse().getRx16Response(rx16);
        	option = rx16.getOption();
        	data = rx16.getData(0);
                sendAddr = rx16.getRemoteAddress16();
                Serial.print("Sender Address (16): "); Serial.println(sendAddr);
                
        } else {
                xbee.getResponse().getRx64Response(rx64);
        	option = rx64.getOption();
        	data = rx64.getData(0);
                sendAddr = rx16.getRemoteAddress16();
                Serial.print("Sender Address (64): "); Serial.println(sendAddr);
        }
      } else {
      	// not something we were expecting
        Serial.println("Unexpected");
      }
    } else if (xbee.getResponse().isError()) {
      Serial.print("Error reading packet.  Error code: ");  
      Serial.println(xbee.getResponse().getErrorCode());
    } 
}
