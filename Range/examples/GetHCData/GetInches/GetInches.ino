/*
  GetInches.ino - Test case for HCSR04 Ultrasonic Sensor
  Created by Taylor Skilling, June 12th, 2017
  Released into the public domain.
*/

#include "HCSR04.h"

// Instantiate sensor oject
Ultrasonic ultrasonic;

void setup(){
  // Set up Serial library at 9600 baud
  Serial.begin(9600);      
  // Use inialization method of Ultrasonic class
  ultrasonic.begin(7,6);
}

void loop(){
  int inches;
  ultrasonic.DistanceMeasure();
  inches = ultrasonic.microsecondsToInches();  
  Serial.print(("Distance in inches: ")); 
  Serial.println(inches);
  delay(1000);
}
