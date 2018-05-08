int PUL=3; //define Pulse pin
int DIR=4; //define Direction pin
int Limit = 23; //define Enable Pin

//accelstepper

int timeDel = 100;
int steps = 1350;

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
    digitalWrite(DIR,HIGH);
    digitalWrite(PUL,HIGH);
    delayMicroseconds(10*timeDel);
    digitalWrite(PUL,LOW);
    delayMicroseconds(10*timeDel);
  }

  if(digitalRead(Limit) == HIGH)
  {
    delay(5000);
    int mult = 1;
    for (int i=0; i<steps; i++)
    {
      if(i < 250){
        mult = 5;
      }
      else{
        mult = 1;
      }
      
      digitalWrite(DIR,LOW);
      digitalWrite(PUL,HIGH);
      delayMicroseconds(timeDel*mult);
      digitalWrite(PUL,LOW);
      delayMicroseconds(timeDel*mult);
    }
    delay(5000);
  }
}
