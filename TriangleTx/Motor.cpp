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
volatile uint8_t remainingDistanceA; 
volatile uint8_t remainingDistanceB;

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

// Function declarations based on EnableInterrupt library
void distanceDecrementA()
{
  remainingDistanceA--;
}
void distanceDecrementB()
{
  remainingDistanceB--;
}

// Function which drives a certain distance, with compensation for shit-tier motors
// Distance is in centimeters
void Motor::driveDistance(uint8_t spd, double distance) 
{
  // Declare variables
  uint8_t wheel_circumference_cm = 22;
  uint8_t num_slots = 20;
  double scale = 0.8;

  // Determine the number of encoder slots needed to pass
  double num_rotations = distance / wheel_circumference_cm;
  uint8_t total_slots = num_rotations * num_slots;

  // Assign to volatile variables, necessary for library
  remainingDistanceA = total_slots;
  remainingDistanceB = total_slots;

  // Enable Interrupt Library
  enableInterrupt(SPDA, distanceDecrementA, RISING);
  enableInterrupt(SPDB, distanceDecrementB, RISING);

  // Begin driving
  driveArdumoto(MOTOR_A, CW, spd);
  driveArdumoto(MOTOR_B, CW, spd);

  // While there's distance left to go!
  while(remainingDistanceA > 0){
    // Check every quarter rotation of the wheel
    if ((remainingDistanceA % (num_slots / 4)) == 0){
      // If the right wheel is turning faster, slow it down, and vice-versa
      if (remainingDistanceA > remainingDistanceB){
        driveArdumoto(MOTOR_B, CW, spd * scale);
      } else if (remainingDistanceA < remainingDistanceB) {
        driveArdumoto(MOTOR_A, CW, spd * scale);
      } else {
        driveArdumoto(MOTOR_A, CW, spd);
        driveArdumoto(MOTOR_B, CW, spd);
      }
    }
  }
  // Stop the robot and disable interrupts  
  stopArdumoto(0);
  stopArdumoto(1);
  disableInterrupt(SPDA);
  disableInterrupt(SPDB);
}