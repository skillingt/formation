#include "Robot.h"
#include "XBee.h"
#include "Wire.h"

Robot botA; // 2

void setup() {
  // Initialize serial 
  Serial.begin(9600);  
  // Call begin function
  botA.init_Robot();
}

void loop() {
  // Populate position struct with a possible position of a detected robot
  // Control bit = 1 = confirmPosition
  // Control bit = 0 = moveToPosition
  botA.pos.control = 1;
  botA.pos.distance = 12;
  botA.pos.bearing1 = 104;
  botA.pos.bearing2 = 1;
  // Cast to uint8_t*
  uint8_t* payload = reinterpret_cast<uint8_t*>(&botA.pos);
  
  /* 
  // Other payload creating methods
  uint8_t payload[4];
  memcpy(payload, &botA.pos, sizeof(botA.pos));
  uint8_t payload[] = {botA.pos.control, botA.pos.distance, botA.pos.bearing};
  */
  
  /* 
  // Debugging Prints
  Serial.println(payload[0]); // control
  Serial.println(payload[1]); // distance
  Serial.println(payload[2]); // bearing
  Serial.println(payload[3]); // bearing
  */
  
  // Send to bot 1
  botA.send(0x2345, payload);
  
  // Delay to prevent interference from subsequent transmission
  delay(15000);
}
