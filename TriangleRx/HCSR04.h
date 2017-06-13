/*
  HCSR04.h - Class for HC-SR04 Ultrasonic Sensor
  Created by Taylor Skilling, June 12th, 2017.
  Released into the public domain.
*/

#include "Arduino.h"

#ifndef HCSR04_h
#define HCSR04_h

class Ultrasonic
{
	public:
		void begin(int pinTrig, int pinEcho);
        void DistanceMeasure(void);
		long microsecondsToInches(void);
	private:
		int _pinTrig; // Pin connected to the sensor's Trigger pin
		int _pinEcho; // Pin connected to the sensor's Echo pin
        long duration; // Received duration of sensor pulse
};

#endif