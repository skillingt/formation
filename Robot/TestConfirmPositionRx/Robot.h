/*
  Robot.cpp - Class header for autonomous formation robots
  Created by Taylor Skilling, April 25th, 2017.
  Released into the public domain.
*/

#include <XBee.h>
#include <Arduino.h>
#include "Wire.h"
#include "Motor.h"
#include "Range.h"
#include "HMC5883L.h"

#ifndef Robot_h
#define Robot_h

// Pin Assignments 
const byte green_LED = 10;  
const byte orange_LED = 9; 
const byte red_LED = 8; 

class Robot
{
  public:
    // Data members
    uint16_t ID;
    struct Position{
      byte control;
      float distance, bearing;
    };
    Position pos; 
    Motor motor;
    Range range;
    HMC5883L mag;
    // Member functions
    void init_Robot();
    bool findObject(Position &pos);
    void flashLed(byte LED);
    bool confirmPosition(Position &pos);
    bool send(uint16_t addr16, uint8_t* payload);
    bool receive(Position &pos);
};

#endif