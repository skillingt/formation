/*
  RadioRange.cpp - Class for RadioShack Ultrasonic Range Sensor
  Catalog #: 2760342
  Created by Taylor Skilling, June 12th, 2017.
  Based off RadioShack Example Code
  Released into the public domain.
*/

#include "RadioRange.h"

void Ultrasonic::begin(int pin)
{
	_pin = pin;
}

void Ultrasonic::DistanceMeasure(void){
	// Measures the distance between the sensor and an object

	// Set the SIG pin as an output
    pinMode(_pin, OUTPUT);
    // Send a pulse to the sensor
	digitalWrite(_pin, LOW);
	delayMicroseconds(2);
	digitalWrite(_pin, HIGH);
	delayMicroseconds(5);
	digitalWrite(_pin,LOW);
	// Set the SIG pin as an input
	pinMode(_pin,INPUT);
	// Measure the duration of the high pulse from the sensor
	duration = pulseIn(_pin,HIGH);
}


long Ultrasonic::microsecondsToCentimeters(void){
	// Returns the measured distance in centimeters
	// Sensor is rated for ranges 0 to 400 centimeters

	// 29 microseconds per centimeter (Speed of sound is 340 m/s)
  	// Distance = time / 29 (μs/in) / wave out and back
	return duration / 29 / 2;	
}

long Ultrasonic::microsecondsToInches(void){
	// Returns the measured distance in inches
	// Sensor is rated for ranges 0 to 157 inches

  	// 73.746 microseconds per inch (1130 feet per second)
  	// Distance = time * 74 (μs/in) / wave out and back
	return duration / 74 / 2;	
}