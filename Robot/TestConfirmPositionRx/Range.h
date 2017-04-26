/*
  Range.cpp - Library for LV-MaxSonar-EZ (MB1030)
  Created by Taylor Skilling, February 27th, 2017.
  Released into the public domain.
*/

#include <Arduino.h>

#ifndef Range_h
#define Range_h

// Pin Assignments //
const byte analog_in = 0;  // Analog Voltage Output  

class Range
{
  public:
    void init_Range();
	int SampleAndAverage();
	float ConvertToInches(int raw_value);
	float ConvertToCentimeters(int rawValue);
	float GetInches();
	float GetCentimeters();
};

#endif