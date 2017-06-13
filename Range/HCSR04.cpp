/*
  HCSR04.cpp - Class for HC-SR04 Ultrasonic Sensor
  Created by Taylor Skilling, June 12th, 2017.
  Released into the public domain.
*/

#include "HCSR04.h"

void Ultrasonic::begin(int pinTrig, int pinEcho)
{
	_pinTrig = pinTrig;
	_pinEcho = pinEcho;
}

void Ultrasonic::DistanceMeasure(void)
{
  // Setup pin modes
	pinMode(_pinTrig, OUTPUT);
  pinMode(_pinEcho, INPUT);
  // Clear the Trigger pin
  digitalWrite(_pinTrig, LOW); 
  delayMicroseconds(2); 
  // Send a pulse to initiate reading
  digitalWrite(_pinTrig, HIGH);
  delayMicroseconds(10); 
  digitalWrite(_pinTrig, LOW);
  // Measure the duration of the high pulse from the sensor
  duration = pulseIn(_pinEcho, HIGH);
}

long Ultrasonic::microsecondsToInches(void){
  // Returns the measured distance in inches
  // Sensor is rated for ranges 0 to 157 inches
  // 73.746 microseconds per inch (1130 feet per second)
  // Distance = time * 74 (μs/in) / wave out and back
	return duration / 74 / 2;	
}