/*
Mag.cpp - Library for HMC583L Compass/Magnetometer
Created by Taylor Skilling, April 14th, 2017.
Based off HMC5883 Library by Kevin Townsend for Adafruit Industries
Released into the public domain.
*/

#include "Mag.h"

Mag::Mag(){
// Use initializer list for Arduino OOD
//Mag::Mag() : magnetometer(Adafruit_HMC5883_Unified(1234)) {
//Mag::Mag() : magnetometer(1234){
	// Begin serial connection
	Serial.begin(9600);

	// Assign a unique ID to the sensor at startup
	magnetometer = Adafruit_HMC5883_Unified(1234);

}

void Mag::begin(void){
  // Initialize the sensor 
  if(!magnetometer.begin())
  {
    // Error detecting the HMC5883 sensor
    Serial.println("Unable to detect HMC5883 sensor");
    while(1);
  }
  else
  {
    Serial.println("Succesfully began");
  }

  // Display basic sensor information on startup
  displaySensorDetails();
}


void Mag::displaySensorDetails(void)
{
  sensor_t sensor;
  magnetometer.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
}


float Mag::getHeading(void) 
{
  // Get a new sensor event 
  sensors_event_t event; 
  magnetometer.getEvent(&event);

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  // Declination angle is the 'Error' of the magnetic field 
  // Boston is -14* 41' W, which is ~14 Degrees, or 0.244 radians
  float declinationAngle = 0.244;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  headingDegrees = heading * 180/M_PI; 

  return headingDegrees;
}