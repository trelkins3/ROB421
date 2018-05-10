#include <Encoder.h>
#include <math.h>

#define MAX_VEL 40
#define MAX_ROT_VEL 0.1
#define R_BODY 15.748

#define KP 5.00
#define KI 0.00

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

#define VEL_SMOOTHING_FACTOR 0.75
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

long i;
bool dir = false;

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

 


  delay(3000);
  
  prevMicros = micros();
  enc1.write(0);
  enc2.write(0);
  enc3.write(0);
  _Enc1_Prev_Count = 0;
  _Enc2_Prev_Count = 0;
  _Enc3_Prev_Count = 0;
  i = 0;
}

void loop() {
  parseSerial();
  updateVelocityEstimates();
  updateDriveMotors();
  

  des_forward_vel = 0.98*des_forward_vel;
  des_right_vel = 0.98*des_right_vel;
  des_rotational_vel = 0.98*des_rotational_vel;
}

void updateDriveMotors(){
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

void parseSerial(){
  if(Serial.available() > 0){
    char received = Serial.read();
    inData += received; 

    // Process message when new line character is recieved
    if ((received == 'X') && (inData.length() <= 5)){
      //Serial.print("Arduino Received: ");
      //Serial.print(inData);
      //Serial.print("\noutput\n");

      int buffer_ = 0;
      int inputSize = inData.length() - 2;
      
      // Convert to 0-999
      if(inputSize == 1)
        buffer_ = inData.charAt(1)-48;
      else if(inputSize == 2)
        buffer_ = ((inData.charAt(1)-48)*10)+(inData.charAt(2)-48);
      else if(inputSize == 3)
        buffer_ = ((inData.charAt(1)-48)*100)+((inData.charAt(2)-48)*10)+(inData.charAt(3)-48);    
      
      //Serial.print("buffer_: ");
      //Serial.print(buffer_);
      if(inData.charAt(0) == 'B'){   
        des_forward_vel = MAX_VEL*(buffer_ - 500)/500.0;
        
        //Serial.print("Read in: ");
        //Serial.print(inData);
      }
      else if((inData.charAt(0) == 'A')){        
        des_right_vel = -MAX_VEL*(buffer_ - 500)/500.0;
        
        //Serial.print("Read in: ");
        //Serial.print(inData);
      }
      else if((inData.charAt(0) == 'C')){ 
        des_rotational_vel = MAX_ROT_VEL*(buffer_ - 500)/500.0;
        
        //Serial.print("Read in: ");
        //Serial.print(inData);
        //Serial.print("New Forward Vel: ");
        //Serial.println(des_rotational_vel);
      
      }
      //else if( inData.charAt(0) == 'D' && inData.length() <= 3){}
      Serial.print(STEP_TO_DIST*(enc1.read())); Serial.print(",");
      Serial.print(STEP_TO_DIST*(enc2.read())); Serial.print(",");
      Serial.print(STEP_TO_DIST*(enc3.read())); Serial.print(",");
      
      Serial.print(_Enc1_Vel); Serial.print(",");
      Serial.print(_Enc2_Vel); Serial.print(",");
      Serial.print(_Enc3_Vel); Serial.print(",");
      
      Serial.print((0.86602*des_forward_vel + 0.5*des_right_vel + R_BODY*des_rotational_vel)); Serial.print(",");
      Serial.print((0.0*des_forward_vel - 1*des_right_vel + R_BODY*des_rotational_vel)); Serial.print(",");
      Serial.print((-0.86602*des_forward_vel + 0.5*des_right_vel + R_BODY*des_rotational_vel)); Serial.print(",");
      
      Serial.print(curr_PWM_1); Serial.print(",");
      Serial.print(curr_PWM_2); Serial.print(",");
      Serial.print(curr_PWM_3); Serial.print("X");
      
      Serial.print(motor_1_integrated); Serial.print(",");
      Serial.print(motor_2_integrated); Serial.print(",");
      Serial.print(motor_3_integrated); Serial.print("X");

      inData = ""; // Clear received buffer_
      updateDriveMotors();
     }
     
   }
   else if(inData.length() > 5)
     inData = "";
}

double sign(double value) { 
 return double((value>0)-(value<0)); 
}
