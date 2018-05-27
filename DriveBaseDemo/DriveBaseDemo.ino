// Quickstop.pde
// -*- mode: C++ -*-
//
// Check stop handling.
// Calls stop() while the stepper is travelling at full speed, causing
// the stepper to stop as quickly as possible, within the constraints of the
// current acceleration.
//
// Copyright (C) 2012 Mike McCauley
// $Id:  $

#include <AccelStepper.h>

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 6, 5); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

int Limit = 23; //define Enable Pin

const int DRIVE_1_PWM_PIN  = 29;
const int DRIVE_1_DIR_A_PIN = 27;
const int DRIVE_1_DIR_B_PIN = 28; 
const int DRIVE_2_PWM_PIN = 30;
const int DRIVE_2_DIR_A_PIN = 31;
const int DRIVE_2_DIR_B_PIN = 32;
const int DRIVE_3_PWM_PIN = 10;
const int DRIVE_3_DIR_A_PIN = 11;
const int DRIVE_3_DIR_B_PIN = 12;

void setup()
{
  pinMode (DRIVE_1_PWM_PIN, OUTPUT);
  pinMode (DRIVE_2_PWM_PIN, OUTPUT);
  pinMode (DRIVE_3_PWM_PIN, OUTPUT);

  stepper.setMaxSpeed(500);
  stepper.setAcceleration(3000);
  pinMode (Limit, INPUT);

  pinMode(DRIVE_1_PWM_PIN,OUTPUT);
  pinMode(DRIVE_1_DIR_A_PIN,OUTPUT);
  pinMode(DRIVE_1_DIR_B_PIN,OUTPUT);
  pinMode(DRIVE_2_PWM_PIN,OUTPUT);
  pinMode(DRIVE_2_DIR_A_PIN,OUTPUT);
  pinMode(DRIVE_2_DIR_B_PIN,OUTPUT);
  pinMode(DRIVE_3_PWM_PIN,OUTPUT);
  pinMode(DRIVE_3_DIR_A_PIN,OUTPUT);
  pinMode(DRIVE_3_DIR_B_PIN ,OUTPUT);
}

void loop()
{

  double power = 100*sin(8*millis()/1000.0);

  
  analogWrite(DRIVE_1_PWM_PIN, abs((int)(power)));
  analogWrite(DRIVE_2_PWM_PIN, abs((int)(power)));
  analogWrite(DRIVE_3_PWM_PIN, abs((int)(power)));

  
  digitalWrite(DRIVE_1_DIR_A_PIN, power/abs(power));
  digitalWrite(DRIVE_1_DIR_B_PIN, -power/abs(power));
  digitalWrite(DRIVE_2_DIR_A_PIN, power/abs(power));
  digitalWrite(DRIVE_2_DIR_B_PIN, -power/abs(power));
  digitalWrite(DRIVE_3_DIR_A_PIN, power/abs(power));
  digitalWrite(DRIVE_3_DIR_B_PIN, -power/abs(power));

}
