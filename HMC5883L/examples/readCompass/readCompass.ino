#include <Wire.h>
#include "HMC5883L.h"


HMC5883L compass;

void setup(){
  // Initialize Serial for debugging
  Serial.begin(9600);
  // Call the begin method
  compass.begin();
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);
  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);
  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(-60, 9);
}

void loop(){
  // Read the magnetometer
  float heading = compass.readCompass();
  // Display the reading over the serial monitor
  Serial.print("Current Heading: "); Serial.println(heading);
  // Wait half a second before reading another value
  delay(500);
}