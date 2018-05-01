int PUL=7; //define Pulse pin
int DIR=6; //define Direction pin
int ENA=5; //define Enable Pin

//accelstepper

int timeDel = 100;
int steps = 1400;

void setup() {
  pinMode (PUL, OUTPUT);
  pinMode (DIR, OUTPUT);
  pinMode (ENA, OUTPUT);

}

void loop() {
  for (int i=0; i<steps; i++)
  {
    digitalWrite(DIR,LOW);
    digitalWrite(ENA,HIGH);
    digitalWrite(PUL,HIGH);
    delayMicroseconds(timeDel);
    digitalWrite(PUL,LOW);
    delayMicroseconds(timeDel);
  }
  delay(5000);
  
  for (int i=0; i<steps; i++)
  {
    digitalWrite(DIR,HIGH);
    digitalWrite(ENA,HIGH);
    digitalWrite(PUL,HIGH);
    delayMicroseconds(timeDel);
    digitalWrite(PUL,LOW);
    delayMicroseconds(timeDel);
  }
delay(5000);
}
