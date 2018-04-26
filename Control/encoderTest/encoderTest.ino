/*
*Quadrature Decoder 
*/
#include "Arduino.h"
#include <digitalWriteFast.h>  // library for high performance reads and writes by jrraines
                               // see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
                               // and http://code.google.com/p/digitalwritefast/

// It turns out that the regular digitalRead() calls are too slow and bring the arduino down when
// I use them in the interrupt routines while the motor runs at full speed.

// Quadrature encoders
// Left encoder
#define c_LeftEncoderInterruptA 0
#define c_LeftEncoderInterruptB 1
#define c_LeftEncoderPinA 2
#define c_LeftEncoderPinB 3
#define LeftEncoderIsReversed
#define PWMoutPin 10

volatile bool _LeftEncoderASet;
volatile bool _LeftEncoderBSet;
volatile bool _LeftEncoderAPrev;
volatile bool _LeftEncoderBPrev;
volatile double _LeftEncoderVelocity = 0;
unsigned long _LeftEncoderPrevTime = 0;
unsigned long _LeftEncoderTime;
long _LeftEncoderPrevCount = 0;
volatile long _LeftEncoderTicks = 0;

void setup()
{
  Serial.begin(250000);

  // Quadrature encoders
  // Left encoder
  pinMode(c_LeftEncoderPinA, INPUT);      // sets pin A as input
  digitalWrite(c_LeftEncoderPinA, LOW);  // turn on pullup resistors
  pinMode(c_LeftEncoderPinB, INPUT);      // sets pin B as input
  digitalWrite(c_LeftEncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(c_LeftEncoderInterruptA, HandleLeftMotorInterruptA, CHANGE);
  attachInterrupt(c_LeftEncoderInterruptB, HandleLeftMotorInterruptB, CHANGE);

  //PWM output
  pinMode(PWMoutPin, OUTPUT);
  
  _LeftEncoderPrevTime = micros();
}

void loop()
{ 
  _LeftEncoderTime = micros();

  //magic number is 1,000,000 micro seconds per second divided by 537.6 counts per rev 
 
  Serial.print("Ticks ");
  Serial.print(_LeftEncoderTicks);
  Serial.print(" vel ");
  Serial.print(_LeftEncoderVelocity);
  
  _LeftEncoderVelocity = 0.95*_LeftEncoderVelocity + 0.05*(60.0*1860.119*(_LeftEncoderTicks - _LeftEncoderPrevCount))/(_LeftEncoderTime - _LeftEncoderPrevTime);

  analogWrite(PWMoutPin,max(min(255 - 10*(int)(100 + _LeftEncoderVelocity) ,255),0));
  
  _LeftEncoderPrevTime = _LeftEncoderTime;
  _LeftEncoderPrevCount = _LeftEncoderTicks;

  Serial.print(" Effort ");
  Serial.print(max(min(255 - 150*(int)(100 + _LeftEncoderVelocity) ,255),0));
  Serial.print("\n");
  
}


// Interrupt service routines for the left motor's quadrature encoder
void HandleLeftMotorInterruptA(){
  _LeftEncoderBSet = digitalReadFast(c_LeftEncoderPinB);
  _LeftEncoderASet = digitalReadFast(c_LeftEncoderPinA);
  
  _LeftEncoderTicks+=ParseEncoder();
  
  _LeftEncoderAPrev = _LeftEncoderASet;
  _LeftEncoderBPrev = _LeftEncoderBSet;
  
  
}

// Interrupt service routines for the right motor's quadrature encoder
void HandleLeftMotorInterruptB(){
  // Test transition;
  _LeftEncoderBSet = digitalReadFast(c_LeftEncoderPinB);
  _LeftEncoderASet = digitalReadFast(c_LeftEncoderPinA);
  
  _LeftEncoderTicks+=ParseEncoder();
  
  _LeftEncoderAPrev = _LeftEncoderASet;
  _LeftEncoderBPrev = _LeftEncoderBSet;
  
}

int ParseEncoder(){
  if(_LeftEncoderAPrev && _LeftEncoderBPrev){
    if(!_LeftEncoderASet && _LeftEncoderBSet) return 1;
    if(_LeftEncoderASet && !_LeftEncoderBSet) return -1;
  }else if(!_LeftEncoderAPrev && _LeftEncoderBPrev){
    if(!_LeftEncoderASet && !_LeftEncoderBSet) return 1;
    if(_LeftEncoderASet && _LeftEncoderBSet) return -1;
  }else if(!_LeftEncoderAPrev && !_LeftEncoderBPrev){
    if(_LeftEncoderASet && !_LeftEncoderBSet) return 1;
    if(!_LeftEncoderASet && _LeftEncoderBSet) return -1;
  }else if(_LeftEncoderAPrev && !_LeftEncoderBPrev){
    if(_LeftEncoderASet && _LeftEncoderBSet) return 1;
    if(!_LeftEncoderASet && !_LeftEncoderBSet) return -1;
  }
}
