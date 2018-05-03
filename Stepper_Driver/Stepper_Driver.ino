int PUL=3; //define Pulse pin
int DIR=4; //define Direction pin
int Limit = 23; //define Enable Pin

//accelstepper

int timeDel = 100;
int steps = 1400;

void setup() {
  pinMode (PUL, OUTPUT);
  pinMode (DIR, OUTPUT);
  pinMode (Limit, INPUT);

}

void loop() 
{
  Serial.println(digitalRead(Limit));
  while(digitalRead(Limit) == LOW)
  {
    digitalWrite(DIR,LOW);
    digitalWrite(PUL,HIGH);
    delayMicroseconds(timeDel);
    digitalWrite(PUL,LOW);
    delayMicroseconds(timeDel);
  }

  if(digitalRead(Limit) == HIGH)
  {
    delay(5000);
    for (int i=0; i<steps; i++)
    {
      digitalWrite(DIR,HIGH);
      digitalWrite(PUL,HIGH);
      delayMicroseconds(timeDel);
      digitalWrite(PUL,LOW);
      delayMicroseconds(timeDel);
    }
    delay(5000);
  }
}
