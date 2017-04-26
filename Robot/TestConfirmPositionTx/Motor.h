/*
  Motor.h - Library for Ardumoto Shield.
  Created by Taylor Skilling, February 24th, 2017.
  Based off Ardumoto Example Sketch by Jim Lindblom
  Released into the public domain.
*/

#include<Arduino.h>

#ifndef Motor_h
#define Motor_h

// Clockwise and counter-clockwise definitions.
// Depending on how you wired your motors, you may need to swap.
#define CW  0
#define CCW 1

// Motor definitions to make life easier:
#define MOTOR_A 0
#define MOTOR_B 1

// Pin Assignments //
// Don't change these! These pins are statically defined by shield layout
const byte PWMA = 3;  // PWM control (speed) for motor A
const byte PWMB = 11; // PWM control (speed) for motor B
const byte DIRA = 12; // Direction control for motor A
const byte DIRB = 13; // Direction control for motor B

class Motor
{
  public:
    void init_Motor();
    void driveArdumoto(byte motor, byte dir, byte spd);
    void rotateArdumotoCW(byte spd);
    void rotateArdumotoCCW(byte spd);
    void stopArdumoto(byte motor);
};

#endif