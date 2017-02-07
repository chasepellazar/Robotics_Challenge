//XBEE 2.4GHZ Transmitter System For Delivering Location Relative Bearing 
//in Degrees. For LSM9DS1 compass system.
//Finalized by Jack Maydan based off Adam St. Amard's earlier versions.
//Edited by Robert Belter 10/30/2015
//Edited by Thomas Horning 02/07/2017

//
//This program works based on the Spark Fun Arduino FIO v3.3 with an XBEE
//transmitter hooked to an extended antennae.
//The board also is hooked to a 3 axis magnetometer. 
//
//The entire module rotates, calculates the bearing based off magnetomer, 
//and transmits it through the patch antennae.
//
//This code is open source and free to use, its derivatives should follow the same guidelines.

#include <XBee.h>
#include <Wire.h>
#include <SparkFunLSM9DS1.h>


//---COMPASS SETUP---

LSM9DS1 imu;
// I2C setup
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
  
//--------------------CALIBRATION FOR MAGNETOMETER---------------------
//In order to ensure that your transmitter will read the correct heading,
//we have provided a calibration mode that will print the values over the
//Xbees. Make sure to use with XB_RX_Calibration


//Uncomment the below line to activate print out over serial for magnetometer x,y,z
//#define calibration_mode

//Uncomment the below line to activate output above ^ over XBee 
//#define output_calibration

//Set Declination angle
#define DECLINATION 8.58

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
  
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // Verification of connection with compass
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while (1);
  }
}

void loop(){
  
  getVectorLSM();

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

void getVectorLSM () {
  float reading = -1;
  float x, y, z;
  imu.readMag();
  // Subtract calculated offsets from magnetometer data
  x = imu.calcMag(imu.mx);
  y = imu.calcMag(imu.my);
  z = imu.calcMag(imu.mz);
  //Adjust values by offsets
  x += xoff;
  y += yoff;
  // Scaling correction
  x *= xscale;
  y *= yscale;

#ifdef calibration_mode
  xout = x;
  yout = y;
  zout = z;
  char output[100];
  sprintf(output, "x: %d, y: %d, z: %d", xout, yout, zout);
  Serial.println(output);
#endif

  // Calculate heading
  float heading;
  if (y == 0)
    heading = (x < 0) ? PI : 0;
  else
    heading = atan2(y, x);

  heading -= DECLINATION * (PI / 180);

  if (heading > 2 * PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += (2 * PI);

  // Convert everything from radians to degrees:
  reading = heading * 180/PI;
  heading_converter.f = reading;    // return the heading or bearing
}
