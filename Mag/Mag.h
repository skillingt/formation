/*
Mag.h - Library for HMC583L Compass/Magnetometer
Created by Taylor Skilling, April 14th, 2017.
Based off HMC5883 Library by Kevin Townsend for Adafruit Industries
Released into the public domain.
*/

#ifndef MAG_H
#define MAG_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

class Mag
{
	private:
		float headingDegrees;
		Adafruit_HMC5883_Unified magnetometer;
	public:
		Mag();
		void begin(void);
		void displaySensorDetails(void);
		float getHeading(void);
};

#endif