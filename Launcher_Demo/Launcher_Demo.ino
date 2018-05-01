#include <Servo.h>
#include <AccelStepper.h>

Servo latch;  //pin 8
AccelStepper arm(1, 9, 10); // Step = 9, Dir = 10
int limit = 32; // limit switch = 32
int stepper = 9; //analog 9
int servo = 8; // analog 8

long stepperPos = 0;
int servoPos = 0;

void setup() 
{
  latch.attach(8);  // attaches the servo on pin 9 to the servo object
  arm.setMaxSpeed(100); //steps per second
  arm.setAcceleration(10); //steps per second ^2

  pinMode(limit, INPUT_PULLUP);
}

void loop() 
{
  //check for button
  if(digitalRead(limit) == LOW)
  {
    Serial.println("zero");
    servoPos = 0;
    stepperPos = 0;
  }
  if(analogRead(stepper) > 550)
  {
    stepperPos++;
  }
  if(analogRead(stepper) < 450)
  {
    stepperPos--;
  }
  if(analogRead(servo) > 550)
  {
    servoPos++;
  }
  if(analogRead(servo) < 450)
  {
    servoPos--;
  }
  
  latch.write(servoPos);
  arm.runToNewPosition(stepperPos);

  delay(20);
   
}
