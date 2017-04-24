#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Mag.h>

Mag mag;

void setup(){
  Serial.begin(9600);
  mag.begin();
}

void loop(){
  // Read the magnetometer
  float heading = mag.getHeading();
  // Display the reading over the serial monitor
  Serial.print("Current Heading: "); Serial.println(heading);
  // Wait half a second before reading another value
  delay(500);
}