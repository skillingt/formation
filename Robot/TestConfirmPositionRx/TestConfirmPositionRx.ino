#include "Robot.h"
#include "XBee.h"
#include <Wire.h>

Robot botB; // 1

int failLed = 8;
int statusLed = 9;
int successLed = 10;

void flashLed(int pin, int times, int wait) {
    
    for (int i = 0; i < times; i++) {
      digitalWrite(pin, HIGH);
      delay(wait);
      digitalWrite(pin, LOW);
      
      if (i + 1 < times) {
        delay(wait);
      }
    }
}

void setup() {
  // Initialize serial 
  Serial.begin(9600); 
  // Call begin function
  botB.init_Robot();
  // put your setup code here, to run once:
  pinMode(failLed, OUTPUT);
  pinMode(statusLed, OUTPUT);
  pinMode(successLed, OUTPUT);
  
  /*
  flashLed(failLed, 10, 100);
  delay(100);
  flashLed(statusLed, 10, 100);
  delay(100);
  flashLed(successLed, 10, 100);
  botB.flashLed(8);
  */
}

void loop() {
  // put your main code here, to run repeatedly:
  bool success;
 
  // Control
  // Receive coordinates from bot 1
  success = botB.receive(botB.pos);
  if (success){
    flashLed(successLed, 2, 200);
  }else{
    flashLed(failLed, 3, 200);
  }
  delay(1000);
  // Confirm position
  if (botB.pos.control == 1){
    flashLed(successLed, 3, 200);
  } else {
    flashLed(failLed, 4, 100);
  }
  
  // Distance
  // Receive coordinates from bot 1
  success = botB.receive(botB.pos);
  if (success){
    flashLed(successLed, 2, 200);
  }else{
    flashLed(failLed, 3, 200);
  }
  delay(1000);
  // See if we have the correct distance
  if (botB.pos.distance > 11.0 && botB.pos.distance < 13.0){
    flashLed(statusLed, 3, 200);
  }else{
    flashLed(failLed, 4, 100);
  }
  
  // Bearing1
  success = botB.receive(botB.pos);
  if (success){
    flashLed(successLed, 2, 200);
  }else{
    flashLed(failLed, 3, 200);
  }
  delay(1000);
  // See if we have the correct bearing1
  if (botB.pos.bearing1 == 104){
    flashLed(successLed, 6, 200);
  }else{
    flashLed(failLed, 4, 100);
  }
  
  // Bearing2
  success = botB.receive(botB.pos);
  if (success){
    flashLed(successLed, 2, 200);
  }else{
    flashLed(failLed, 3, 200);
  }
  delay(1000);
  // See if we have the correct bearing2
  if (botB.pos.bearing2 == 1){
    flashLed(statusLed, 6, 200);
  }else{
    flashLed(failLed, 4, 100);
  }
  
  // Put together the two bearings into one cohesive
  uint16_t intermediate = 0x0000;
  intermediate = botB.pos.bearing2;
  intermediate = intermediate << 8;
  intermediate |= botB.pos.bearing1;
  botB.pos.bearing = intermediate;
  
  if (botB.pos.bearing == 360){
    flashLed(successLed, 10, 200);
    // Confirm Position
    botB.confirmPosition(botB.pos);
    delay(1000);
    flashLed(successLed, 4, 200);
  }else{
    flashLed(failLed, 10, 100);
  }
}
