#include "Arduino.h"

#ifndef range3_h
#define range3_h

class Ultrasonic
{
	public:
		Ultrasonic(int pinTrig, int pinEcho);
        void DistanceMeasure(void);
		long microsecondsToInches(void);
	private:
		int _pinTrig;
		int _pinEcho;//pin number of Arduino that is connected with SIG pin of Ultrasonic Ranger.
        long duration;// the Pulse time received;
};

#endif