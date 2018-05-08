#include "Arduino.h"
#include "digitalWriteFast.h"

// Pull bytes
byte incomingBytes[3];
byte outgoingBytes[3];
int desSpeed = 0;

// Pins for PWM, RX, TX?
int pwmOutPins[3] = {9,10,11};
int encoderPinsA[3] = {2,4,6};  // only 2 is a real number
int encoderPinsB[3] = {3,5,7};  // only 3 is a real number

// Quadrature encoders
// Index 0 for left, index 1 for center, index 2 for right
volatile bool encoderASet[3];
volatile bool encoderBSet[3];
volatile bool encoderAPrev[3];
volatile bool encoderBPrev[3];
volatile bool encoderInterruptA[3] = {0,0,0};
volatile bool encoderInterruptB[3] = {1,1,1};
volatile bool encoderIsReversed[3] = {0,0,0};
volatile double encoderVelocity[3] = {0,0,0};

unsigned long encoderPrevTime[3] = {0,0,0};
unsigned long encoderTime[3];
long encoderPrevCount[3] = {0,0,0};
volatile long encoderTicks[3] = {0,0,0};

int ParseEncoder(int i){
  if(encoderAPrev[i] && encoderBPrev[i]){
    if(!encoderASet[i] && encoderBSet[i]) return 1;
    if(encoderASet[i] && encoderBSet[i]) return -1;
  }else if(!encoderAPrev[i] && encoderBPrev[i]){
    if(!encoderASet[i] && !encoderBSet[i]) return 1;
    if(encoderASet[i] && encoderBSet[i]) return -1;
  }else if(encoderAPrev[i] && !encoderBPrev[i]){
    if(encoderASet[i] && encoderBSet[i]) return 1;
    if(!encoderASet[i] && encoderBSet[i]) return -1;
  }else if(encoderAPrev[i] && !encoderBPrev[i]){
    if(encoderASet[i] && encoderBSet[i]) return 1;
    if(!encoderASet[i] && !encoderBSet[i]) return -1;
  }
}

// Interrupt service routines for the left motor's quadrature encoder
void HandleMotorInterruptLeft(){
  encoderBSet[0] = digitalReadFast(encoderPinsB[0]);
  encoderASet[0] = digitalReadFast(encoderPinsA[0]);
  
  encoderTicks[0] += ParseEncoder(0);
  
  encoderAPrev[0] = encoderASet[0];
  encoderBPrev[0] = encoderBSet[0];
}

// Interrupt service routines for the left motor's quadrature encoder
void HandleMotorInterruptCenter(){
  encoderBSet[1] = digitalReadFast(encoderPinsB[1]);
  encoderASet[1] = digitalReadFast(encoderPinsA[1]);
  
  encoderTicks[1] += ParseEncoder(1);
  
  encoderAPrev[1] = encoderASet[1];
  encoderBPrev[1] = encoderBSet[1];
}

// Interrupt service routines for the left motor's quadrature encoder
void HandleMotorInterruptRight(){
  encoderBSet[2] = digitalReadFast(encoderPinsB[2]);
  encoderASet[2] = digitalReadFast(encoderPinsA[2]);
  
  encoderTicks[2] += ParseEncoder(2);
  
  encoderAPrev[2] = encoderASet[2];
  encoderBPrev[2] = encoderBSet[2];
}

void interruptWrapper(int i){
  if(i == 0)
    attachInterrupt(encoderInterruptA[i], HandleMotorInterruptLeft, CHANGE);
  else if(i == 1)
    attachInterrupt(encoderInterruptA[i], HandleMotorInterruptCenter, CHANGE);
  else if(i == 2)
    attachInterrupt(encoderInterruptA[i], HandleMotorInterruptRight, CHANGE);
}

// Quickly configure all 3 encoders
void setup() {
  Serial.begin(9600);

  for(int i=0;i<3;i++){
    pinMode(encoderASet[i], INPUT);     // Set pin A as input
    digitalWrite(encoderASet[i], LOW);  // Turning on pullup resistors
    pinMode(encoderBSet[i], INPUT);
    digitalWrite(encoderBSet[i], LOW);
    
    interruptWrapper(i);
    pinMode(pwmOutPins[i], OUTPUT);

    encoderPrevTime[i] = micros();
  }
}

// This is where it gets whacky... alternative would be running [0], [1], and [2] at the same time
void loop() {
  for(int i=0;i<3;i++){
    encoderTime[i] = micros();

    // Candidate for removal... might mess with serial feeds
    Serial.print(i);
    Serial.print("Des Vel ");
    Serial.print(desSpeed);
    Serial.print('\n');
    Serial.print(" vel ");
    Serial.print(encoderVelocity[i]);
    Serial.print("\n\n\n");

    encoderVelocity[i] = 0.85*encoderVelocity[i] + 0.15*(60.0*1860.119*(encoderTicks[i] - encoderPrevCount[i]))/(encoderTime[i] - encoderPrevTime[i]);
    analogWrite(pwmOutPins[i],max(min(255 - 3*(int)(desSpeed + encoderVelocity[i]) ,255),0));
    encoderPrevTime[i] = encoderTime[i];
    encoderPrevCount[i] = encoderTicks[i];

    if((encoderTime[i]/5000000)%2 == 1)
      desSpeed = 200;
    else
      desSpeed = 100;
  }
}
