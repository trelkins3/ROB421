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
AccelStepper stepper(AccelStepper::DRIVER,3,4); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

int Limit = 23; //define Enable Pin

void setup()
{  
  stepper.setMaxSpeed(500);
  stepper.setAcceleration(3000);
  pinMode (Limit, INPUT);
}

void loop()
{    

  // Now go backwards
  stepper.setAcceleration(1000); 
  stepper.setMaxSpeed(300);
  stepper.moveTo(40000);
  while (digitalRead(Limit) == LOW) // Full speed basck to 0
    stepper.run();
  stepper.stop(); // Stop as fast as possible: sets new target
  stepper.setCurrentPosition(0); 
  // Now stopped after quickstop

  delay(5000); //Chill out

  //Crank it
  stepper.setMaxSpeed(90000);
  stepper.setAcceleration(10000); 
  stepper.moveTo(-2000); //Go way past
  while (stepper.currentPosition() >= -1250) // stop at this point
    stepper.run();
  stepper.setAcceleration(500000);  //Allow us to stop as fast as possible
  stepper.stop(); // Stop as fast as possible: sets new target
  stepper.runToPosition(); 
  // Now stopped after quickstop
  
  delay(2000); //Wait for vibrations to chill out



}
