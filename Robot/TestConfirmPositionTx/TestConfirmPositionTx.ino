#include "Robot.h"
#include "XBee.h"
#include "Wire.h"

Robot botA; // 2

void setup() {
  // put your setup code here, to run once
}

void loop() {
  // Populate position struct with a possible position of a detected robot
  // Control bit = 1 = confirmPosition
  // Control bit = 0 = moveToPosition
  botA.pos.control = 1;
  botA.pos.distance = 12.0;
  botA.pos.bearing = 270.0;
  uint8_t payload[] = {botA.pos.control, botA.pos.distance, botA.pos.bearing};
  
  // Send to bot 1
  botA.send(0x2345, payload);

}
