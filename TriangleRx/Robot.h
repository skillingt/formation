/*
  Robot.cpp - Class header for autonomous formation robots
  Created by Taylor Skilling, April 25th, 2017.
  Released into the public domain.
*/
#include <Arduino.h>

#include <XBee.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Motor.h"
#include "HCSR04.h"

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
      uint8_t control;
      uint8_t distance;
      uint8_t bearing1;
      uint8_t bearing2;
      uint16_t bearing;
    };
    Position pos; 
    Motor motor;
    Adafruit_BNO055 bno;
    Ultrasonic ultrasonic;
    // Member functions
    void init_Robot();
    void rotateToBearing(Position &pos);
    bool findObject(Position &pos);
    void flashLed(byte LED);
    bool confirmPosition(Position &pos);
    bool send(uint16_t addr16, uint8_t* payload);
    bool sendPosition(uint16_t addr16, Position &pos);
    bool receiveConfirmation();
    bool receive(Position &pos);
    void packStruct(Position &pos);
    void findTriangle(Position &pos, Position &pos2, Position &pos3);
    // Helper functions for angles and findTriangle
    float anglediff(float a, float b);
    float subdeg(float a, float b);
    float addeg(float a, float b);
    float backAngle(float a);
    float betaDirSelect(float brgMOV, float brgREF, float beta);
};

#endif