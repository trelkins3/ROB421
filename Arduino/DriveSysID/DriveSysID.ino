#include <Encoder.h>

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
 // Loop Execution Time
  long PWM = 2*(i/500);
  if(PWM > 250){
    i = 0;
    PWM = 0;
    dir = !dir;
  }
  unsigned long Td = micros() - prevMicros;

  // Update filtered velocities
  _Enc1_Vel = VEL_SMOOTHING_FACTOR*_Enc1_Vel + (1 - VEL_SMOOTHING_FACTOR)*STEP_TO_DIST*(enc1.read() - _Enc1_Prev_Count)/(Td*0.000001);
  _Enc1_Prev_Count = enc1.read();
  _Enc2_Vel = VEL_SMOOTHING_FACTOR*_Enc2_Vel + (1 - VEL_SMOOTHING_FACTOR)*STEP_TO_DIST*(enc2.read() - _Enc2_Prev_Count)/(Td*0.000001);
  _Enc2_Prev_Count = enc2.read();
  _Enc3_Vel = VEL_SMOOTHING_FACTOR*_Enc3_Vel + (1 - VEL_SMOOTHING_FACTOR)*STEP_TO_DIST*(enc3.read() - _Enc3_Prev_Count)/(Td*0.000001);
  _Enc3_Prev_Count = enc3.read();

  analogWrite(DRIVE_1_PWM,PWM);
  analogWrite(DRIVE_2_PWM,PWM);
  analogWrite(DRIVE_3_PWM,PWM);
  
  if( dir == true){
    digitalWrite(DRIVE_1_DIR_A,LOW);
    digitalWrite(DRIVE_1_DIR_B,HIGH);
    digitalWrite(DRIVE_2_DIR_A,LOW);
    digitalWrite(DRIVE_2_DIR_B,HIGH);
    digitalWrite(DRIVE_3_DIR_A,LOW);
    digitalWrite(DRIVE_3_DIR_B,HIGH);
    PWM = -PWM;
  }
  else{ 
    digitalWrite(DRIVE_1_DIR_A,HIGH);
    digitalWrite(DRIVE_1_DIR_B,LOW);
    digitalWrite(DRIVE_2_DIR_A,HIGH);
    digitalWrite(DRIVE_2_DIR_B,LOW);
    digitalWrite(DRIVE_3_DIR_A,HIGH);
    digitalWrite(DRIVE_3_DIR_B,LOW);
  }


  Serial.print("t,");
  Serial.print(prevMicros+Td);
  Serial.print(",PWM,");
  Serial.print(PWM);
  Serial.print(",Enc1,");
  Serial.print(_Enc1_Vel);
  Serial.print(",Enc2,");
  Serial.print(_Enc2_Vel);
  Serial.print(",Enc3,");
  Serial.println(_Enc3_Vel);

  prevMicros = Td + prevMicros;
  delay(10);
  i++;
}
