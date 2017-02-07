//XBEE 2.4GHZ Transmitter System For Delivering Location Relative Bearing in Degrees.
//Finalized by Jack Maydan based off Adam St. Amard's earlier versions.
//Edited by Robert Belter 10/30/2015

//
//This program works based on the Spark Fun Arduino FIO v3.3 with an XBEE transmitter hooked to an extended antennae.
//The board also is hooked to a 3 axis magnetometer. 
//
//The entire module rotates, calculates the bearing based off magnetomer, 
//and transmits it through the patch antennae.
//
//This code is open source and free to use, its derivatives should follow the same guidelines.

#include <XBee.h>
#include <Wire.h>
//#include <HMC5883L.h>
#define address 0x1E 

//--------------------CALIBRATION FOR MAGNETOMETER---------------------
//In order to ensure that your transmitter will read the correct heading,
//we have provided a calibration mode that will print the values over the
//Xbees. Make sure to use with XB_RX_Calibration

//Uncomment the below line to activate print out over serial for magnetometer x,y,z
//#define calibration_mode

//Uncomment the below line to activate output above ^ over XBee 
//#define output_calibration

//Axis offsets for magnetometer
int xoff = -7;
int yoff = 54;
int zoff = 0;

//Axis scales for magnetometer
float xscale = 1.070;
float yscale = 1.117;
float zscale = 1;

#ifdef output_calibration
union{
  int i[3];
  uint8_t b[6];
}calibration_converter;
#endif

//Current readings from magnetometer axis
int xout, yout, zout; 

//-----------------------END CALIBRATION FOR MAGNETOMETER------------------

XBee xbee = XBee();
int compassAddress = 0x42 >> 1;

uint8_t payload[12];
int payload_size = 4;


union{
  float f;
  uint8_t b[4];
}heading_converter;

void setup(){
  Wire.begin();
  Serial.begin(57600);
  Serial1.begin(57600);
  xbee.setSerial(Serial1);
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
}

void loop(){
  getVector();
  Serial.print("Theta: ");
  Serial.println(heading_converter.f, 2); // print the heading/bearing

  //Create payload
  makePayload();
  
  //Address of receiving device can be anything while in broadcasting mode
  Tx16Request tx = Tx16Request(0x5678, payload, payload_size);
  xbee.send(tx);
  
  //Delay must be longer than the readPacket timeout on the receiving module
  delay(10);
}

void makePayload(){
#ifndef output_calibration
  //Copy heading into payload
  memcpy(payload, heading_converter.b, 4);
#else
  //Use the calibration values as the output
  calibration_converter.i[0] = xout;
  calibration_converter.i[1] = yout;
  calibration_converter.i[2] = zout;
  memcpy(payload, calibration_converter.b, 12);
  payload_size = 12;
#endif
}


/*--------------------------------------------------------------
This the the fucntion which gathers the heading from the compass.
----------------------------------------------------------------*/
void getVector () {
  float reading = -1;
  int x,y,z;

  // step 1: instruct sensor to read echoes 
  Wire.beginTransmission(address);  // transmit to device
  // the address specified in the datasheet is 66 (0x42) 
  // but i2c adressing uses the high 7 bits so it's 33 
  Wire.write(0x03);          // command sensor to measure angle  
  Wire.endTransmission();  // stop transmitting 

  // step 3: request reading from sensor 
  Wire.requestFrom(address, 6);

  // step 4: receive reading from sensor 
  if(6 <= Wire.available())     // if two bytes were received 
  { 
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb/
    y |= Wire.read(); //Y lsb
   
  } 
  //Adjust values by offsets
  x += xoff;
  y += yoff;
  z += zoff;
  //Scale axes
  x *= xscale;
  y *= yscale;
  z *= zscale;
  
#ifdef calibration_mode
  xout = x;
  yout = y;
  zout = z;
  char output[100];
  sprintf(output, "x: %d, y: %d, z: %d", xout, yout, zout);
  Serial.println(output);
#endif

  float heading = atan2(y,x);
  heading += PI/2;
  if(heading < 0)
    heading += 2*PI;
  if(heading > 2*PI)
    heading -= 2*PI;
  reading = heading * 180/PI;
  heading_converter.f = reading;    // return the heading or bearing
}

