void setup() { pinMode(11, OUTPUT); }

void loop() {
  int i;

  // Spin motor up
  for(i = 255; i > 0; i--){
    delay(100);
    analogWrite(11, i);
  }

  // Spin motor back down
  for(i = 0; i < 255; i++){
    delay(100);
    analogWrite(11,i);
  }
}
