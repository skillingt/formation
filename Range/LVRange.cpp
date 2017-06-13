/* 
  Range.cpp - Library for LV-MaxSonar-EZ (MB1030)
  Created by Taylor Skilling, February 27th, 2017.
  Released into the public domain.
  Three useful functions are defined:
    SampleAndAverage() -- Samples the sensor multiple times and averages.
    ConvertToInches(raw_value) -- Convert the averaged sensor readings to inches.
    GetInches() -- Returns the sensor distance in inches.
  The last two functions above also exist for centimeters.
*/

#include "LVRange.h"

Range::Range()
{
  // All pins should be setup as outputs:
  pinMode(analog_in, INPUT);

  // Initialize all pins as low:
  //digitalWrite(analog_in, LOW);

  // Initialize serial 
  Serial.begin(9600); 
}

// SampleAndAverage takes a number of distance readings and averages them, returning a raw value
int Range::SampleAndAverage()
{
  int raw_value, i, num_samples;
  raw_value = 0;
  num_samples = 8;
  // Sample
  for (i = 0; i < num_samples; i++) {
    raw_value += analogRead(analog_in);
    delay(50);
  }
  // Average
  raw_value /= num_samples;

  return raw_value;
}

// ConvertToInches takes a raw sensor value and convert it to a distance in inches
float Range::ConvertToInches(int raw_value)
{
  int supply_voltage;
  float voltage, scale_factor, inches;
  // Convert the analog reading to a voltage
  voltage = raw_value * (5.0 / 1023.0);
  // Sensor is powered from 5V Arduino pin
  supply_voltage = 5.0;
  // "Scaling factor is (Vcc/512) per inch"
  scale_factor = supply_voltage / 512.0;
  // Convert 
  inches = 0.0;
  inches = voltage / scale_factor;
  return inches;
}

// ConvertToCentimeters takes a raw sensor value and convert it to a distance in centimeters
float Range::ConvertToCentimeters(int raw_value)
{
  int supply_voltage;
  float voltage, scale_factor, centimeters;
    // Convert the analog reading to a voltage
  voltage = raw_value * (5.0 / 1023.0);
  // Sensor is powered from 5V Arduino pin
  supply_voltage = 5.0;
  // "Scaling factor is (Vcc/512) per inch"
  scale_factor = supply_voltage / 512.0;
  // Convert
  centimeters = 0.0;
  centimeters = voltage / scale_factor * 2.54;

  return centimeters;
}

// GetInches returns the distance in inches
float Range::GetInches()
{
  float raw_value;
  raw_value = SampleAndAverage();
  return ConvertToInches(raw_value);
}

// GetCentimeters returns the distance in centimeters
float Range::GetCentimeters()
{
  float raw_value;
  raw_value = SampleAndAverage();
  return ConvertToCentimeters(raw_value);
}