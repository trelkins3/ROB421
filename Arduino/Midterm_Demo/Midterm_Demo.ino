//#include <i2c_t3.h>
#include <Adafruit_TCS34725.h>

Adafruit_TCS34725 color = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

uint16_t r; uint16_t g; uint16_t b; uint16_t c;

int lightPin = 0;  //define a pin for Photo resistor (needs to be analog in!)
int ledPin = 11;     //define a pin for LED
int init_brightness = 1000;
int brightness = 0;

void setup()
{
  Serial.begin(9600);
  Serial.println("Color View Test!");

  pinMode( ledPin, OUTPUT );

  // detect initial brightness values
  init_brightness = analogRead(lightPin);

  // loop through and initalize all sensors
  if (color.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
  
}
void loop()
{
   brightness = init_brightness - analogRead(lightPin); // get the realtive brightness
    
  //Serial.println(brightness); //Write the value of the photoresistor to the serial monitor.

  // determine if the brightness is low enough for a ball to be caught
  if(brightness >= 70)
  {
    // turn on the led
    digitalWrite(ledPin, HIGH);
    Serial.println("Caught Ball");
  }
  else
  {
    digitalWrite(ledPin, LOW);
  }

  // read the color sensor
  color.getRawData(&r, &g, &b, &c);

  //look for blue
  if((c>170) && (r<30) && (g>60) && (b>96))
  {
    Serial.println("Line");
    // turn on the led
    digitalWrite(ledPin, HIGH);
    Serial.println("Caught Ball");
  }

//    Serial.print("\tC:\t"); Serial.print(c);
//    Serial.print("\tR:\t"); Serial.print(r);
//    Serial.print("\tG:\t"); Serial.print(g);
//    Serial.print("\tB:\t"); Serial.println(b);  
}
