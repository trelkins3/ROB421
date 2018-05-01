int lightPin = 0;  //define a pin for Photo resistor (needs to be analog in!)
int ledPin = 11;     //define a pin for LED
int init_brightness = 1000;
int brightness = 0;

void setup()
{
    Serial.begin(9600);  //Begin serial communcation
    pinMode( ledPin, OUTPUT );

    // detect initial brightness values
    init_brightness = analogRead(lightPin);
}

void loop()
{
    brightness = init_brightness - analogRead(lightPin); // get the realtive brightness
    
    Serial.println(brightness); //Write the value of the photoresistor to the serial monitor.

    // determine if the brightness is low enough for a ball to be caught
    if(brightness >= 50)
    {
      // turn on the led
      digitalWrite(ledPin, LOW);
    }
    else
    {
      digitalWrite(ledPin, HIGH);
    }
}
