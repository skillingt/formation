/* 
  Motor.cpp - Library for Ardumoto Shield
  Created by Taylor Skilling, February 24th, 2017.
  Based off Ardumoto Example Sketch by Jim Lindblom
  Released into the public domain.

  Three useful functions are defined:
    init_Motor() -- Setup the Ardumoto Shield pins
    driveArdumoto([motor], [direction], [speed]) -- Drive [motor] 
      (0 for A, 1 for B) in [direction] (0 or 1) at a [speed]
      between 0 and 255. It will spin until told to stop.
    stopArdumoto([motor]) -- Stop driving [motor] (0 or 1).
*/

#include "Motor.h"

// Set by driveDistance() and modified by distanceDecrement()
volatile int remDistanceA; 
// remDistance units = slots on encoder = 20/22 cm
volatile int remDistanceB;

void Motor::init_Motor()
{
  // All pins should be setup as outputs:
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);

  // Initialize all pins as low:
  digitalWrite(PWMA, LOW);
  digitalWrite(PWMB, LOW);
  digitalWrite(DIRA, LOW);
  digitalWrite(DIRB, LOW);
}


// driveArdumoto drives 'motor' in 'dir' direction at 'spd' speed
void Motor::driveArdumoto(byte motor, byte dir, byte spd)
{
  if (motor == MOTOR_A)
  {
    digitalWrite(DIRA, dir);
    analogWrite(PWMA, spd);
  }
  else if (motor == MOTOR_B)
  {
    digitalWrite(DIRB, dir);
    analogWrite(PWMB, spd);
  }  
}

// rotateArdumoto makes the robot rotate CW at a desired speed
void Motor::rotateArdumotoCW(byte spd)
{
  digitalWrite(DIRA, CCW);
  analogWrite(PWMA, spd);
  digitalWrite(DIRB, CW);
  analogWrite(PWMB, spd);
}

// rotateArdumoto makes the robot rotate CCW at a desired speed
void Motor::rotateArdumotoCCW(byte spd)
{
  digitalWrite(DIRA, CW);
  analogWrite(PWMA, spd);
  digitalWrite(DIRB, CCW);
  analogWrite(PWMB, spd);
}

// stopArdumoto makes a motor stop
void Motor::stopArdumoto(byte motor)
{
  driveArdumoto(motor, 0, 0);
}

void distanceDecrementA()
{
  remDistanceA--;
}
void distanceDecrementB()
{
  remDistanceB--;
}

// Function which drives a certain distance, with compensation for shit-tier motors
// Distance in cm. Spd has to be about 120-150.
void Motor::driveDistanceStr8(byte spd, float distance) 
{
  int countA = 0;
  int countB = 0;
  driveArdumoto(MOTOR_A, CW, spd);
  driveArdumoto(MOTOR_B, CW, spd);
  float numWheels = distance/22;
  int numSlots = numWheels*20;
  remDistanceA = numSlots;
  remDistanceB = numSlots;
  enableInterrupt(SPDA, distanceDecrementA, RISING);
  enableInterrupt(SPDB, distanceDecrementB, RISING);
  while(remDistanceA>0){
    delay(1);
    if ((remDistanceA % 12) == 0){
      if (remDistanceA > remDistanceB) {
        driveArdumoto(MOTOR_B, CW, spd*.6);
        countB=0;
      }
      if (remDistanceA < remDistanceB) {
        driveArdumoto(MOTOR_A, CW, spd*.6);
        countA=0;
      }
    }
    if (countA==10) driveArdumoto(MOTOR_A, CW, spd);
    if (countB==10) driveArdumoto(MOTOR_B, CW, spd);
    countA++;
    countB++;
  }
  disableInterrupt(SPDA);
  disableInterrupt(SPDB);
}