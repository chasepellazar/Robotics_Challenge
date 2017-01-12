
void compassSetup(){
  
  compass.settings.device.commInterface = IMU_MODE_I2C;
  compass.settings.device.mAddress = LSM9DS1_M;
  compass.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // compass.begin(), which verifies communication with the compass
  // and turns it on.
  if (!compass.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }
}

float getHeading(){
  
  float x,y;
  
  // Once you have your heading, you must then add your 'Declination Angle', 
  // which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/ Mine is:
  // +8° 29' West, which is 8.483 Degrees, or (which we need) 0.14805628 radians, I will use 8.483 
  // degrees since we convert it to Radians later in the code.
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off
  float declinationAngle = 8.483; //Declination in Boulder, Colorado

  // Calibration parameters for accuracy

  float Xoffset = -.12;
  float Yoffset = 0.52;
  float Xscale = 0.98;
  float Yscale = 1.02;
  // Get Magnetic field readings
  compass.readMag();
  
  // Subtract calculated offsets from magnetometer data
  x = compass.calcMag(compass.mx)-Xoffset;
  y = compass.calcMag(compass.my)-Yoffset;

  // Scaling correction
  x *= Xscale;
  y *= Yscale;
  
  // Begin to calculate heading
  float heading;

  // Calculate the angle
  if (y == 0)
    heading = (x < 0) ? PI : 0;
  else
    heading = atan2(y,x);

  // Correct for Declination
    heading -= declinationAngle * (PI/180);

  // Correct for sign errors
  if (heading > 2*PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += (2 * PI);
  
  // Convert everything from radians to degrees:
  heading *= (180.0 / PI);

  //Return the heading
  return heading;
  //Serial.print("Heading: "); Serial.println(heading, 2);
}

