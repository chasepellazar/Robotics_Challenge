// Wire Master Reader
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Reads data from an I2C/TWI slave device
// Refer to the "Wire Slave Sender" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>

void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
}

void loop()
{
  Wire.requestFrom(8, 2);    // request 6 bytes from slave device #8
  Serial.println(getHeading());
  delay(500);
}

int getHeading(){
  Wire.requestFrom(8, 2);    // request 6 bytes from slave device #8
  byte MSB = Wire.read();
  byte LSB = Wire.read();
  return (MSB<<8)|LSB;
}

