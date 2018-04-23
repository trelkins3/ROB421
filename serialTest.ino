void setup() {
  Serial.begin(9600);
  pinMode(22, INPUT);
  pinMode(24, INPUT);
  pinMode(26, INPUT);
  pinMode(28, INPUT);
  pinMode(30, INPUT);
  pinMode(32, INPUT);
  pinMode(34, INPUT);
  pinMode(36, INPUT);
}

void loop() {
  Serial.print(digitalRead(22));
  Serial.print(" ");
  Serial.print(digitalRead(24));
  Serial.print(" ");
  Serial.print(digitalRead(26));
  Serial.print(" ");
  Serial.print(digitalRead(28));
  Serial.print(" ");
  Serial.print(digitalRead(30));
  Serial.print(" ");
  Serial.print(digitalRead(32));
  Serial.print(" ");
  Serial.print(digitalRead(34));
  Serial.print(" ");
  Serial.print(digitalRead(36));
  Serial.println();
}
