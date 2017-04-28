#include "range3.h"

Ultrasonic::Ultrasonic(int pinTrig, int pinEcho)
{
	_pinTrig = pinTrig;
	_pinEcho = pinEcho;
}

void Ultrasonic::DistanceMeasure(void)
{
	pinMode(_pinTrig, OUTPUT);
  	pinMode(_pinEcho, INPUT);

    	digitalWrite(_pinTrig, LOW);  // Added this line
  	delayMicroseconds(2); // Added this line
  	digitalWrite(_pinTrig, HIGH);
	//  delayMicroseconds(1000); - Removed this line
  	delayMicroseconds(10); // Added this line
  	digitalWrite(_pinTrig, LOW);
  	duration = pulseIn(_pinEcho, HIGH);
}


/*The measured distance from the range 0 to 157 Inches*/
long Ultrasonic::microsecondsToInches(void)
{
	return duration/74/2;	
}


