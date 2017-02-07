//This code is for testing the triple axis magnetometer HMC5883L. It will print out readings from your magnetometer. A
//properly functioning compass should read approximately 0-359 when pointed in the proper direction. 
//0 degrees is north, 90 east, etc. 
//Beware of magnetic field interference (nearby wires, bolts, etc.)
//Test outside away from any electronics
//Jack Maydan 10/8/15
#include <HMC5883L.h>

    #include <Wire.h> //I2C Arduino Library
#define address 0x1E //0011110b, I2C 7bit address of HMC5883 

void setup(){
  //Initialize Serial and I2C communications
  Serial.begin(9600);
  Wire.begin();
  
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
}
  void loop(){
  
  int x,y,z; //triple axis data

  //Tell the HMC5883L where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
 
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }

//  MagnetometerRaw raw = Wire.ReadRawAxis();
   float heading = atan2(y,x);
   if(heading < 0)
      heading += 2*PI;
   float headingDegrees = heading * 180/M_PI;
    Serial.print("Theta: ");
   Serial.println(headingDegrees);
  
  //Print out values of each axis
  Serial.print("x: ");
  Serial.print(x);
  Serial.print("  y: ");
  Serial.println(y);
  
  delay(750);
}
