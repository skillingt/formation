#include "Range.h"

Range range;

void setup(){
	// Set up Serial library at 9600 baud
	Serial.begin(9600);           
}

void loop(){
	float inches;
	inches = range.GetInches();
	Serial.print(F("Distance in inches: ")); 
        Serial.println(inches);
	delay(1000);
}
