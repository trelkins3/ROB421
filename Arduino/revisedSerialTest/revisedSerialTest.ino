const double MAX_VEL = 1;
const int MAX_ROT_VEL = 1;
const double R_BODY = 40;
const double R_WHEEL = 4;

const int DRIVE_1_PWM_PIN  = 29;
const int DRIVE_1_DIR_A_PIN = 27;
const int DRIVE_1_DIR_B_PIN = 28; 
const int DRIVE_2_PWM_PIN = 30;
const int DRIVE_2_DIR_A_PIN = 31;
const int DRIVE_2_DIR_B_PIN = 32;
const int DRIVE_3_PWM_PIN = 10;
const int DRIVE_3_DIR_A_PIN = 11;
const int DRIVE_3_DIR_B_PIN = 12;


String inData = "";

double des_forward_vel = 0;
double des_right_vel = 0;
double des_rotational_vel = 0;


void setup(){
  Serial.begin(57600);
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

void updateDriveMotors(){
  double motor_1_speed = (1/R_WHEEL)*(0.86602*des_forward_vel + 0.5*des_right_vel + R_BODY*des_rotational_vel);
  double motor_2_speed = (1/R_WHEEL)*(0.0*des_forward_vel - 1*des_right_vel + R_BODY*des_rotational_vel);
  double motor_3_speed = (1/R_WHEEL)*(-0.86602*des_forward_vel + 0.5*des_right_vel + R_BODY*des_rotational_vel);
  
  
  /*Serial.print(" Motor fwd: ");
  Serial.print(des_forward_vel);
  Serial.print(" Motor right: ");
  Serial.print(des_right_vel);
  Serial.print(" Motor rot: ");
  Serial.println(des_rotational_vel);*/
  
  /*Serial.print(" Motor 1: ");
  Serial.print(motor_1_speed);
  Serial.print(" Motor 2: ");
  Serial.print(motor_2_speed);
  Serial.print(" Motor 3: ");
  Serial.println(motor_3_speed);*/
  
  
  analogWrite(DRIVE_1_PWM_PIN,100*abs(motor_1_speed));
  analogWrite(DRIVE_2_PWM_PIN,100*abs(motor_2_speed));
  analogWrite(DRIVE_3_PWM_PIN,100*abs(motor_3_speed));
  
  digitalWrite(DRIVE_1_DIR_A_PIN, motor_1_speed > 0);
  digitalWrite(DRIVE_1_DIR_B_PIN, motor_1_speed <= 0);
  digitalWrite(DRIVE_2_DIR_A_PIN, motor_2_speed > 0);
  digitalWrite(DRIVE_2_DIR_B_PIN, motor_2_speed <= 0);
  digitalWrite(DRIVE_3_DIR_A_PIN, motor_3_speed > 0);
  digitalWrite(DRIVE_3_DIR_B_PIN, motor_3_speed <= 0); 
}

void loop(){
  if(Serial.available() > 0){
    char received = Serial.read();
    inData += received; 

    // Process message when new line character is recieved
    if ((received == 'X') && (inData.length() <= 5)){
      //Serial.print("Arduino Received: ");
      //Serial.print(inData);
      //Serial.print("\noutput\n");

      int buffer = 0;
      int inputSize = inData.length() - 2;
      
      // Convert to 0-999
      if(inputSize == 1)
        buffer = inData.charAt(1)-48;
      else if(inputSize == 2)
        buffer = ((inData.charAt(1)-48)*10)+(inData.charAt(2)-48);
      else if(inputSize == 3)
        buffer = ((inData.charAt(1)-48)*100)+((inData.charAt(2)-48)*10)+(inData.charAt(3)-48);    
      
      //Serial.print("buffer: ");
      //Serial.print(buffer);
      if(inData.charAt(0) == 'A'){   
        des_forward_vel = MAX_VEL*(buffer - 500)/500.0;
        
        //Serial.print("Read in: ");
        //Serial.print(inData);
      }
      else if((inData.charAt(0) == 'B')){        
        des_right_vel = MAX_VEL*(buffer - 500)/500.0;
        
        //Serial.print("Read in: ");
        //Serial.print(inData);
      }
      else if((inData.charAt(0) == 'C')){ 
        des_rotational_vel = MAX_ROT_VEL*(buffer - 500)/500.0;
        
        //Serial.print("Read in: ");
        //Serial.print(inData);
        //Serial.print("New Forward Vel: ");
        //Serial.println(des_rotational_vel);
      
      }
      else if(inData.charAt(0) == 'D'){
        Serial.print(inData);  
      }
      
      //Serial.print("test");
      inData = ""; // Clear received buffer
      updateDriveMotors();
     }
     
   }
   else if(inData.length() > 5)
     inData = "";
}
