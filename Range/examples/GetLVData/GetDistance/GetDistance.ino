#include "LVRange.h"

Range range;

void setup(){
	// Set up Serial library at 9600 baud
	Serial.begin(9600);           
}

void loop(){
	float inches, centimeters;
	inches = range.GetInches();
	centimeters = range.GetCentimeters();
	Serial.print("Distance in inches: "); 
	Serial.println(inches);
	Serial.print("Distance in centimeters: ");
	Serial.println(centimeters);
	delay(1000);
}
