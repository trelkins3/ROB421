// I will now attempt to make my own libarary  for the TCS34725
#include <i2c_t3.h>

#define TCS34725_ADDRESS          (0x29)
#define TCS34725_COMMAND_BIT      (0x80)
#define TCS34725_ENABLE           (0x00)
#define TCS34725_ENABLE_AEN       (0x02)    /* RGBC Enable - Writing 1 actives the ADC, 0 disables it */
#define TCS34725_ENABLE_PON       (0x01)    /* Power on - Writing 1 activates the internal oscillator, 0 disables it */
#define TCS34725_ATIME            (0x01)    /* Integration time */                      
#define TCS34725_CONTROL          (0x0F)    /* Set the gaine level for the sensor */
#define TCS34725_ID               (0x12)    /* 0x44 = TCS34721/TCS34725, 0x4D = TCS34723/TCS34727 */
#define TCS34725_CDATAL           (0x14)    /* Clear channel data */
#define TCS34725_CDATAH           (0x15)
#define TCS34725_RDATAL           (0x16)    /* Red channel data */
#define TCS34725_RDATAH           (0x17)
#define TCS34725_GDATAL           (0x18)    /* Green channel data */
#define TCS34725_GDATAH           (0x19)
#define TCS34725_BDATAL           (0x1A)    /* Blue channel data */
#define TCS34725_BDATAH           (0x1B)

long r; 
long rBase[3]; 
long g; 
long gBase[3];
long b; 
long bBase[3]; 
long c;
long cBase[3]; 
bool line; // true if there is a line detected.

void setup()
{
  Serial.begin(57600);

  // loop through and initalize all sensors
  beg();
  
  // initialize the RGB sensors
  for (int i = 0; i < 3; i++)
  {
    write8(TCS34725_ENABLE, TCS34725_ENABLE_PON, i);
    delay(3);
    write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN, i);
  }

  line = false;

  //get initial values
  for(int i = 0; i < 3; i++)
  {
    getRawData(&r, &g, &b, &c, i);
    rBase[i] = r;
    gBase[i] = g;
    bBase[i] = b;
    cBase[i] = c;
  }
}

void loop()
{
  // loop through and print the data from each sensor
  for(int i = 0; i < 3; i++)
  {
    getRawData(&r, &g, &b, &c, i);

    // normalize
    r = r - rBase[i];
    g = g - gBase[i];
    b = b - bBase[i];
    c = c - cBase[i];
    
//    Serial.print("BASE:\t"); Serial.print(i);
//    Serial.print("\tR:\t"); Serial.print(rBase[i]);
//    Serial.print("\tG:\t"); Serial.print(gBase[i]);
//    Serial.print("\tB:\t"); Serial.println(bBase[i]);

    Serial.print("Sensor:\t"); Serial.print(i);
    Serial.print("\tR:\t"); Serial.print(r);
    Serial.print("\tG:\t"); Serial.print(g);
    Serial.print("\tB:\t"); Serial.print(b);
    Serial.print("\tC:\t"); Serial.println(c);

    //check for line
    if((r > 15000 && g < 25000 && b < 20000 && (c > 8000 && c < 20000)))
    {
      line = true;
      Serial.println(line);
    }
  }  
  delay(300);
}

/**************************************************************************/
/*!
    Helper functions for the Color Sensor
*/
/**************************************************************************/
//Initializes I2C and configures the sensor (call this function before
//  doing anything else)
void beg(void) 
{
  // initalize scl/sda
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 1000);
  Wire.setDefaultTimeout(200000);

  Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_37_38, I2C_PULLUP_EXT, 1000);
  Wire1.setDefaultTimeout(200000);

  Wire2.begin(I2C_MASTER, 0x00, I2C_PINS_3_4, I2C_PULLUP_EXT, 1000);
  Wire2.setDefaultTimeout(200000);
  
  /* Make sure we're actually connected */
  uint8_t x = read8(TCS34725_ID, 0);
  uint8_t y = read8(TCS34725_ID, 1);
  uint8_t z = read8(TCS34725_ID, 2);

  if (((x != 0x44) && (x != 0x10)) || ((y != 0x44) && (y != 0x10)) || ((z != 0x44) && (z != 0x10)))
  {
      Serial.println("TCS34725 ERROR: ");
      
      // traceback checking each read independently
      if((x != 0x44) && (x != 0x10))
      {
        Serial.println("on 0.");
      }
      if((y != 0x44) && (y != 0x10))
      {
        Serial.println("on 1.");
      }
      if((z != 0x44) && (z != 0x10))
      {
        Serial.println("on 2.");
      }
  }
}

//Reads the raw red, green, blue and clear channel values
void getRawData (long *r, long *g, long *b, long *c, int i)
{
  *c = read16(TCS34725_CDATAL, i);
  *r = read16(TCS34725_RDATAL, i);
  *g = read16(TCS34725_GDATAL, i);
  *b = read16(TCS34725_BDATAL, i);
  delay(3); // intigration time
}

//Writes a register and an 8 bit value over I2C
void write8 (uint8_t reg, uint32_t value, int i)
{
  if (i == 0)
  {
    Wire.beginTransmission(TCS34725_ADDRESS);
    Wire.write(TCS34725_COMMAND_BIT | reg);
    Wire.write(value & 0xFF);
    Wire.endTransmission();
}

  if (i == 1)
  {
    Wire1.beginTransmission(TCS34725_ADDRESS);
    Wire1.write(TCS34725_COMMAND_BIT | reg);
    Wire1.write(value & 0xFF);
    Wire1.endTransmission();
  }

  if (i == 2)
  {
    Wire2.beginTransmission(TCS34725_ADDRESS);
    Wire2.write(TCS34725_COMMAND_BIT | reg);
    Wire2.write(value & 0xFF);
    Wire2.endTransmission();
  }
}

//Reads an 8 bit value over I2C
uint8_t read8(uint8_t reg, int i)
{
  if (i == 0)
  {
    Wire.beginTransmission(TCS34725_ADDRESS);
    Wire.write(TCS34725_COMMAND_BIT | reg);
    Wire.endTransmission();
    Wire.requestFrom(TCS34725_ADDRESS, 1);
    return Wire.read();
  }

  if (i == 1)
  {
    Wire1.beginTransmission(TCS34725_ADDRESS);
    Wire1.write(TCS34725_COMMAND_BIT | reg);
    Wire1.endTransmission();
    Wire1.requestFrom(TCS34725_ADDRESS, 1);
    return Wire1.read();
  }

  if (i == 2)
  {
    Wire2.beginTransmission(TCS34725_ADDRESS);
    Wire2.write(TCS34725_COMMAND_BIT | reg);
    Wire2.endTransmission();
    Wire2.requestFrom(TCS34725_ADDRESS, 1);
    return Wire2.read();
  }
  else
    return 0;
}

//Reads a 16 bit values over I2C
long read16(uint8_t reg, int i)
{
  long x; long t;

  if (i == 0)
  {
    Wire.beginTransmission(TCS34725_ADDRESS);
    Wire.write(TCS34725_COMMAND_BIT | reg);
    Wire.endTransmission();
    Wire.requestFrom(TCS34725_ADDRESS, 2);
    t = Wire.read();
    x = Wire.read();
  }

  else if (i == 1)
  {
    Wire1.beginTransmission(TCS34725_ADDRESS);
    Wire1.write(TCS34725_COMMAND_BIT | reg);
    Wire1.endTransmission();
    Wire1.requestFrom(TCS34725_ADDRESS, 2);
    t = Wire1.read();
    x = Wire1.read();
  }

  else if (i == 2)
  {
    Wire2.beginTransmission(TCS34725_ADDRESS);
    Wire2.write(TCS34725_COMMAND_BIT | reg);
    Wire2.endTransmission();
    Wire2.requestFrom(TCS34725_ADDRESS, 2);
    t = Wire2.read();
    x = Wire2.read();
  }
  else
  {
    t = 0;
    x = 0;
  }

  x <<= 8;
  x |= t;
  return x;
}

