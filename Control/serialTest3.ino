void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.write(45);

  int bytesSent = Serial.write("hello");
}
