//#include <i2c_t3.h>
#include <Adafruit_TCS34725.h>

Adafruit_TCS34725 color = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

uint16_t r; uint16_t g; uint16_t b; uint16_t c;

void setup()
{
  Serial.begin(9600);
  Serial.println("Color View Test!");

  // loop through and initalize all sensors
  if (color.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }

//  // initalize scl/sda
//  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 4700);
//  Wire.setDefaultTimeout(200000);
//
//  Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_37_38, I2C_PULLUP_EXT, 4700);
//  Wire1.setDefaultTimeout(200000);
//  
//  Wire2.begin(I2C_MASTER, 0x00, I2C_PINS_3_4, I2C_PULLUP_EXT, 4700);
//  Wire2.setDefaultTimeout(200000);

  
}
void loop()
{
  
  // loop through and initalize all sensors
  for(int i = 0; i < 3; i++)
  {
    color.getRawData(&r, &g, &b, &c, i);

    Serial.print("Sensor:\t"); Serial.print(i);
    Serial.print("\tC:\t"); Serial.print(c);
    Serial.print("\tR:\t"); Serial.print(r);
    Serial.print("\tG:\t"); Serial.print(g);
    Serial.print("\tB:\t"); Serial.println(b);
  }  
}
