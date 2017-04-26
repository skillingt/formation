#include "Robot.h"
#include "XBee.h"
#include "Wire.h"

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
  // Call begin function
  botB.init_Robot();
  // put your setup code here, to run once:
  pinMode(failLed, OUTPUT);
  pinMode(statusLed, OUTPUT);
  pinMode(successLed, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  bool success;
  
  // Receive coordinates from bot 1
  success = botB.receive(botB.pos);
  
  if (success){
    flashLed(successLed, 4, 200);
  }else{
    flashLed(failLed, 3, 200);
  }
  
  delay(1000);
  
  // See if we have the correct data
  if (botB.pos.distance > 11.0 && botB.pos.distance < 13.0){
    flashLed(successLed, 4, 200);
  }else{
    flashLed(failLed, 4, 100);
  }
  
  delay(3000);
  
  // Confirm position
  if (botB.pos.control == 1){
    flashLed(successLed, 4, 200);
    delay(1000);
    // Confirm Position
    botB.confirmPosition(botB.pos);
    delay(1000);
    flashLed(successLed, 4, 200);
  } else {
    flashLed(failLed, 4, 100);
  }
}
