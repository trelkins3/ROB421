#define ENCODER_OPTIMIZE_INTERRUPTS // optimized interrupts.. if cause issue, remove line
#include <Encoder.h>
#include <math.h>
#include <AccelStepper.h>

#define MAX_VEL 60
#define MAX_ROT_VEL 2
#define R_BODY 15.748

#define KP 20.00
#define KI 8.00

#define DRIVE_1_PWM 29
#define DRIVE_1_DIR_A 27
#define DRIVE_1_DIR_B 28
#define ENC_1_A 36
#define ENC_1_B 35

#define DRIVE_2_PWM 30
#define DRIVE_2_DIR_A 31
#define DRIVE_2_DIR_B 32
#define ENC_2_A 34
#define ENC_2_B 33

#define DRIVE_3_PWM 10
#define DRIVE_3_DIR_A 11
#define DRIVE_3_DIR_B 12
#define ENC_3_A 25
#define ENC_3_B 26

#define STEPPER_STEP_PIN 5
#define STEPPER_DIR_PIN 6
#define STEPPER_ENABLE_PIN 13
#define MOTOR_ENABLE_INPUT 14
#define LIMIT 23

#define VEL_SMOOTHING_FACTOR 0.92
// 10.2101761242 inch per rev
// per
// 537.6 steps per output revolution
#define STEP_TO_DIST 0.01899214308


double des_forward_vel = 0;
double des_right_vel = 0;
double des_rotational_vel = 0;

double motor_1_integrated = 0;
double motor_2_integrated = 0;
double motor_3_integrated = 0;

double curr_PWM_1 = 0;
double curr_PWM_2 = 0;
double curr_PWM_3 = 0;

Encoder enc1(ENC_1_A,ENC_1_B);
double _Enc1_Vel = 0;
long _Enc1_Prev_Count = 0;

Encoder enc2(ENC_2_A,ENC_2_B);
double _Enc2_Vel = 0;
long _Enc2_Prev_Count = 0;

Encoder enc3(ENC_3_A,ENC_3_B);
double _Enc3_Vel = 0;
long _Enc3_Prev_Count = 0;

unsigned long prevMicros;
unsigned long loopPeriod;

String inData = "";


enum throwerMechanismState{
readyToThrow,
throwing,
stopping,
retracting
};

AccelStepper stepper(AccelStepper::DRIVER,STEPPER_STEP_PIN,STEPPER_DIR_PIN); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
bool throwCommanded;
throwerMechanismState throwState;
long throwerTimer;

void setup() {   
  Serial.begin(115200);
  
  pinMode(DRIVE_1_PWM, OUTPUT); 
  pinMode(DRIVE_1_DIR_A, OUTPUT); 
  pinMode(DRIVE_1_DIR_B, OUTPUT); 
  
  pinMode(DRIVE_2_PWM, OUTPUT); 
  pinMode(DRIVE_2_DIR_A, OUTPUT); 
  pinMode(DRIVE_2_DIR_B, OUTPUT); 
  
  pinMode(DRIVE_3_PWM, OUTPUT); 
  pinMode(DRIVE_3_DIR_A, OUTPUT); 
  pinMode(DRIVE_3_DIR_B, OUTPUT); 
  
  pinMode (LIMIT, INPUT);

  pinMode(MOTOR_ENABLE_INPUT, INPUT);
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);
  stepper.setMaxSpeed(500);
  stepper.setAcceleration(3000);

  delay(30);
  
  prevMicros = micros();
  enc1.write(0);
  enc2.write(0);
  enc3.write(0);
  _Enc1_Prev_Count = 0;
  _Enc2_Prev_Count = 0;
  _Enc3_Prev_Count = 0;
}

void loop() {
  parseSerial();
  updateVelocityEstimates();
  updateDriveMotors();
  updateThrower();
  
  digitalWrite(STEPPER_ENABLE_PIN,!digitalRead(MOTOR_ENABLE_INPUT));
  des_forward_vel = 0.999*des_forward_vel;
  des_right_vel = 0.999*des_right_vel;
  des_rotational_vel = 0.999*des_rotational_vel;
}

void updateDriveMotors()
{  
  double motor_1_speed = (0.86602*des_forward_vel + 0.5*des_right_vel + R_BODY*des_rotational_vel);
  double motor_2_speed = (0.0*des_forward_vel - 1*des_right_vel + R_BODY*des_rotational_vel);
  double motor_3_speed = (-0.86602*des_forward_vel + 0.5*des_right_vel + R_BODY*des_rotational_vel);

  motor_1_integrated = motor_1_integrated + (motor_1_speed - _Enc1_Vel)*(loopPeriod*0.000001);
  motor_2_integrated = motor_2_integrated + (motor_2_speed - _Enc2_Vel)*(loopPeriod*0.000001);
  motor_3_integrated = motor_3_integrated + (motor_3_speed - _Enc3_Vel)*(loopPeriod*0.000001);

  curr_PWM_1 = min(255,max(-255,getFeedForward( motor_1_speed, 1) + KP*(motor_1_speed - _Enc1_Vel) + KI*motor_1_integrated));
  curr_PWM_2 = min(255,max(-255,getFeedForward( motor_2_speed, 2) + KP*(motor_2_speed - _Enc2_Vel) + KI*motor_2_integrated));
  curr_PWM_3 = min(255,max(-255,getFeedForward( motor_3_speed, 3) + KP*(motor_3_speed - _Enc3_Vel) + KI*motor_3_integrated));
  
  analogWrite(DRIVE_1_PWM,abs(curr_PWM_1));
  analogWrite(DRIVE_2_PWM,abs(curr_PWM_2));
  analogWrite(DRIVE_3_PWM,abs(curr_PWM_3));
  
  digitalWrite(DRIVE_1_DIR_A, curr_PWM_1 > 0);
  digitalWrite(DRIVE_1_DIR_B, curr_PWM_1 <= 0);
  digitalWrite(DRIVE_2_DIR_A, curr_PWM_2 > 0);
  digitalWrite(DRIVE_2_DIR_B, curr_PWM_2 <= 0);
  digitalWrite(DRIVE_3_DIR_A, curr_PWM_3 > 0);
  digitalWrite(DRIVE_3_DIR_B, curr_PWM_3 <= 0); 
}

void updateVelocityEstimates(){
  loopPeriod = micros() - prevMicros;

  // Update filtered velocities
  _Enc1_Vel = VEL_SMOOTHING_FACTOR*_Enc1_Vel + (1 - VEL_SMOOTHING_FACTOR)*STEP_TO_DIST*(enc1.read() - _Enc1_Prev_Count)/(loopPeriod*0.000001);
  _Enc1_Prev_Count = enc1.read();
  _Enc2_Vel = VEL_SMOOTHING_FACTOR*_Enc2_Vel + (1 - VEL_SMOOTHING_FACTOR)*STEP_TO_DIST*(enc2.read() - _Enc2_Prev_Count)/(loopPeriod*0.000001);
  _Enc2_Prev_Count = enc2.read();
  _Enc3_Vel = VEL_SMOOTHING_FACTOR*_Enc3_Vel + (1 - VEL_SMOOTHING_FACTOR)*STEP_TO_DIST*(enc3.read() - _Enc3_Prev_Count)/(loopPeriod*0.000001);
  _Enc3_Prev_Count = enc3.read();

  prevMicros = prevMicros + loopPeriod;
}

void parseSerial(){
  if(Serial.available() > 0){
    char received = Serial.read();
    inData += received; 

    // Process message when new line character is recieved
    if ((received == 'X') && (inData.length() <= 5)){
      //Serial.write("Arduino Received: ");
      //Serial.write(inData);
      //Serial.write("\noutput\n");

      int buffer_ = 0;
      int inputSize = inData.length() - 2;
      
      // Convert to 0-999
      if(inputSize == 1)
        buffer_ = inData.charAt(1)-48;
      else if(inputSize == 2)
        buffer_ = ((inData.charAt(1)-48)*10)+(inData.charAt(2)-48);
      else if(inputSize == 3)
        buffer_ = ((inData.charAt(1)-48)*100)+((inData.charAt(2)-48)*10)+(inData.charAt(3)-48);    
      
      //Serial.write("buffer_: ");
      //Serial.write(buffer_);
      if(inData.charAt(0) == 'B'){   
        des_forward_vel = -MAX_VEL*(buffer_ - 500)/500.0;
        
        //Serial.write("Read in: ");
        //Serial.write(inData);
      }
      else if((inData.charAt(0) == 'A')){        
        des_right_vel = -MAX_VEL*(buffer_ - 500)/500.0;
        
        //Serial.write("Read in: ");
        //Serial.write(inData);
      }
      else if((inData.charAt(0) == 'C')){ 
        des_rotational_vel = MAX_ROT_VEL*(buffer_ - 500)/500.0;
        
        //Serial.write("Read in: ");
        //Serial.write(inData);
        //Serial.write("New Forward Vel: ");
        //Serial.writeln(des_rotational_vel);
      
      }
      else if((inData.charAt(0) == 'D')){
        throwCommanded = (buffer_ > 0);
      }
      //else if( inData.charAt(0) == 'D' && inData.length() <= 3){}
//      Serial.print(STEP_TO_DIST*(enc1.read())); Serial.print(",");
//      Serial.print(STEP_TO_DIST*(enc2.read())); Serial.print(",");
//      Serial.print(STEP_TO_DIST*(enc3.read())); Serial.print(",");
//      
//      Serial.print(_Enc1_Vel); Serial.print(",");
//      Serial.print(_Enc2_Vel); Serial.print(",");
//      Serial.print(_Enc3_Vel); Serial.print(",");
//      
//      Serial.print((0.86602*des_forward_vel + 0.5*des_right_vel + R_BODY*des_rotational_vel)); Serial.print(",");
//      Serial.print((0.0*des_forward_vel - 1*des_right_vel + R_BODY*des_rotational_vel)); Serial.print(",");
//      Serial.print((-0.86602*des_forward_vel + 0.5*des_right_vel + R_BODY*des_rotational_vel)); Serial.print(",");
//
//      Serial.print(des_forward_vel); Serial.print(",");
//      Serial.print(des_right_vel); Serial.print(",");
//      Serial.print(des_rotational_vel); Serial.print(",");
//      
//      Serial.print(curr_PWM_1); Serial.print(",");
//      Serial.print(curr_PWM_2); Serial.print(",");
//      Serial.print(curr_PWM_3); Serial.print(",");
//      
//      Serial.print(motor_1_integrated); Serial.print(",");
//      Serial.print(motor_2_integrated); Serial.print(",");
//      Serial.print(motor_3_integrated); Serial.print(",");
//      
//      Serial.print(throwState); Serial.println("X");

      inData = ""; // Clear received buffer_
     }
     
   }
   else if(inData.length() > 5)
     inData = "";
}

void updateThrower(){
  //Finite State Machine Represenation of throwing mechanism control
  //Handle Transitions first

  // Throw if mechanism is ready to fire and command flag is true 
  if( throwState == readyToThrow && throwCommanded == true){
    stepper.setMaxSpeed(100000);
    stepper.setAcceleration(40000); 
    stepper.moveTo(-6000); //Go way past

    //Serial.println("Began Throw");
    throwState = throwing;
  }
  else if( throwState == throwing && stepper.currentPosition() < -1100){
    stepper.setAcceleration(500000);  //Allow us to stop as fast as possible
    stepper.moveTo(-500); // Stop as fast as possible: sets new target

    throwerTimer = millis();
    
    //Serial.println("Stop Throw");
    throwState = stopping;
  }
  else if( throwState == stopping && millis() - throwerTimer > 400 ){
    stepper.setAcceleration(1000); 
    stepper.setMaxSpeed(1000);
    stepper.moveTo(40000);
    
    //Serial.println("Began Retracting");
    throwState = retracting;
  }
  else if( throwState == retracting && digitalRead(LIMIT) == HIGH){
    stepper.setAcceleration(600000);
    stepper.stop(); // Stop as fast as possible: sets new target
    stepper.setCurrentPosition(0); //Reset stepper count to zero
    stepper.moveTo(-100);

    //Serial.println("Retracted");
    throwState = readyToThrow;
  }
  throwCommanded = false;

  //Update the stepper Command
  stepper.run();
}

/**************************************************************************/
/*!
    Helper functions
*/
/**************************************************************************/
//Note this does not apply any limits, 
//if you ask for a velocity about the max possible 
//it will give you a pwm over 255
double getFeedForward(double velocity, int motor){
  if(velocity == 0)
    return 0;

  //Magic numbers are from system ID fit
  switch(motor){
    case 1:
      return sign(velocity)*( 13.54*exp(0.04429*abs(velocity)) + 4.571e-06*exp(0.3245*abs(velocity)));
    break;
    case 2:
      return sign(velocity)*( 13.4*exp(0.03807*abs(velocity)) + 0.0002331*exp(0.2497*abs(velocity)));
    break;
    case 3:
      return sign(velocity)*( 11.45*exp(0.0437*abs(velocity)) + 4.874e-06*exp(0.3278*abs(velocity))); 
    break;
    default:
      return 0;
    break;
  }
}
double sign(double value) { 
 return double((value>0)-(value<0)); 
}

