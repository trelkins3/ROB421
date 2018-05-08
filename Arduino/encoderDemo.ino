#include "Arduino.h"
#include "digitalWriteFast.h"

// This'll later be changed potentially to an array? Or not
byte incomingByte = 0;


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

void setup(){
  Serial.begin(9600);
  
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

// Will eventullay need 8: one for each sector
void left(){};
void up(){};
void right(){};
void down(){};

// Each of the possible states (only did left and up as per our
// test footage)
void loop(){
  _LeftEncoderTime = micros();
  
  if(Serial.available() > 0){
    incomingByte = Serial.read();
    
    // Candidate for removal
    Serial.print("Ticks ");
    Serial.print(_LeftEncoderTicks);
    Serial.print('\n');
    Serial.print(" vel ");
    Serial.print(_LeftEncoderVelocity);
    Serial.print('\n');
    
    if(incomingByte == '0'){
      _LeftEncoderVelocity = 0.95*_LeftEncoderVelocity + 0.05*(60.0*1860.119*(_LeftEncoderTicks - _LeftEncoderPrevCount))/(_LeftEncoderTime - _LeftEncoderPrevTime);

      analogWrite(PWMoutPin,max(min(100 - 10*(int)(100 + _LeftEncoderVelocity) ,255),0));
  
      _LeftEncoderPrevTime = _LeftEncoderTime;
      _LeftEncoderPrevCount = _LeftEncoderTicks;
      delay(500);
    }
    else if(incomingByte == '1'){
      _LeftEncoderVelocity = 0.95*_LeftEncoderVelocity + 0.05*(60.0*1860.119*(_LeftEncoderTicks - _LeftEncoderPrevCount))/(_LeftEncoderTime - _LeftEncoderPrevTime);

      analogWrite(PWMoutPin,max(min(200 - 10*(int)(100 + _LeftEncoderVelocity) ,255),0));
  
      _LeftEncoderPrevTime = _LeftEncoderTime;
      _LeftEncoderPrevCount = _LeftEncoderTicks;
      delay(500);
    }
    else{
      _LeftEncoderVelocity = 0.95*_LeftEncoderVelocity + 0.05*(60.0*1860.119*(_LeftEncoderTicks - _LeftEncoderPrevCount))/(_LeftEncoderTime - _LeftEncoderPrevTime);

      analogWrite(PWMoutPin,max(min(50 - 10*(int)(100 + _LeftEncoderVelocity) ,255),0));
  
      _LeftEncoderPrevTime = _LeftEncoderTime;
      _LeftEncoderPrevCount = _LeftEncoderTicks;
      delay(500);
    }
  }
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
