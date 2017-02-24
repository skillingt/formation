#include <Motor.h>
#include <avr/sleep.h>

Motor motor;

void setup()
{
}

void loop(){
  // Drive motor A (and only motor A) at various speeds, then stop.
  motor.driveArdumoto(MOTOR_A, CCW, 255); // Set motor A to CCW at max
  delay(1000);  // Motor A will spin as set for 1 second
  motor.driveArdumoto(MOTOR_A, CW, 127);  // Set motor A to CW at half
  delay(1000);  // Motor A will keep trucking for 1 second 
  motor.stopArdumoto(MOTOR_A);  // STOP motor A 

  // Drive motor B (and only motor B) at various speeds, then stop.
  motor.driveArdumoto(MOTOR_B, CCW, 255); // Set motor B to CCW at max
  delay(1000);  // Motor B will spin as set for 1 second
  motor.driveArdumoto(MOTOR_B, CW, 127);  // Set motor B to CW at half
  delay(1000);  // Motor B will keep trucking for 1 second
  motor.stopArdumoto(MOTOR_B);  // STOP motor B 

  // Now spin both!
  motor.driveArdumoto(MOTOR_A, CW, 255);  // Motor A at max speed.
  motor.driveArdumoto(MOTOR_B, CW, 255);  // Motor B at max speed.
  delay(1000);  // Drive forward for a second
  // Now go backwards at half that speed:
  motor.driveArdumoto(MOTOR_A, CCW, 127);  // Motor A at max speed.
  motor.driveArdumoto(MOTOR_B, CCW, 127);  // Motor B at max speed.

  // Rotate the robot
  motor.rotateArdumoto(127);
  delay(5000);
  
  // Stop the motors
  motor.stopArdumoto(0);
  motor.stopArdumoto(1);
  delay(5000);
  
  // Put the Arduino to sleep
  goodnight();
}

void goodnight(){
  cli();
  sleep_enable();
  sleep_cpu();
}

