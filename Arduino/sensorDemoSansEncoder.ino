byte incomingByte = 0;

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}

void left(){};
void up(){};
void right(){};
void down(){};

void loop() {
  /*  serial chunk, read into stateArr  */
  if(Serial.available() > 0){
    incomingByte = Serial.read();
  
    // If left quadrant
    if(incomingByte == '0'){
      digitalWrite(13,0);
      delay(1000);
      digitalWrite(13,255);
    } // Now top sector
    else if(incomingByte == '1'){
      digitalWrite(13,150);
      delay(1000);
      digitalWrite(13,255);
    }
  }
  else
    Serial.print("No out of bounds motion detected.");
}
