/*
  RadioRange.h - Class for RadioShack Ultrasonic Range Sensor
  Catalog #: 2760342
  Created by Taylor Skilling, June 12th, 2017.
  Based off RadioShack Example Code
  Released into the public domain.
*/

#include "Arduino.h"

#ifndef RadioRange_h
#define RadioRange_h

class Ultrasonic
{
	public:
		void begin(int pin);
    void DistanceMeasure(void);
		long microsecondsToCentimeters(void);
		long microsecondsToInches(void);
	private:
		int _pin; // Pin connected to Ultrasonic sensor's SIG pin
    long duration; // Received duration of sensor pulse
};

#endif