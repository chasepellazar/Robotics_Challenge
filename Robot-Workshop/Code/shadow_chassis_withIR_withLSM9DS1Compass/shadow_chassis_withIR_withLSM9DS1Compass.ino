/*

 Title:  shadow_chassis_withIR_withLSM9DS1Compass

 Description:
 Using the Shaddow Chassis, with Arduino Moto driver shied, IR sensor and LSM9DS1 Compass.  Detect obstacles and back away.

 Version 2.0  Jan 2017
 */

#include <Wire.h>
#include <SparkFunLSM9DS1.h>

//Library
LSM9DS1 compass;  // Storing the features in the HMC5883 library as a variable called compass

// I2C setup

// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

//Defining Arduino Pin Assignments

//A is the Right Wheel and B is the Left Wheel
int pwm_a = 3;   // Right Wheel Speed
int pwm_b = 11;  // Left Wheel Speed
int dir_a = 12;  // Right Wheel Direction
int dir_b = 13;  // Left Wheel Direction


int IRsensorPin = A0;    // Define the input pin for the IR sensor
int IRsensorValue = 0;   // Variable to store the Integer value coming from the IR sensor

int LEDForward = 4 ;         // Status LED for moving forward
int LEDBackward = 5 ;        // Status LED for moving backward
int LEDRight = 6 ;           // Status LED for moving right
int LEDLeft = 7 ;           // Status LED for moving left

int movingPin = HIGH;       //Enable movement of the robot.  Low for sensor/Serial work, High moving around (Movement Enabled)
int MoveSize = 100;
int obstacleSeen = 1000;  // value when the robot IR Sensor will trip and it should back up
int speedOfRightWheel = 255;  // The speed of the right wheel
int speedOfLeftWheel = 255;   // The speed of the left wheel

float currentHeading = 0.0;  // Variable to hold Heading in degrees


void setup()
{
  Serial.begin(9600);
  Wire.begin();                // join i2c bus

  pinMode(pwm_a, OUTPUT);      //Set motor control pins to be outputs from the Arduino
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);

  //LED setups
  pinMode(LEDForward, OUTPUT);    //Moving forward
  pinMode(LEDBackward, OUTPUT);   //Moving backwards
  pinMode(LEDRight, OUTPUT);      //turning right
  pinMode(LEDLeft, OUTPUT);       //turning left

  //Input pins
  pinMode(IRsensorPin, INPUT);   // Reading from the IR sensor
  //pinMode(movingPin, INPUT);     // Enables movement but still collects compass and IR readings and reports to serial

  analogWrite(pwm_a, 0);          //set both motors speed to 0% duty cycle (off)
  analogWrite(pwm_b, 0);

  compassSetup();                // setup the HMC5883L
}

void loop()
{

  //turn all leds off at beginning of loop
  digitalWrite(LEDForward, LOW);    //forwards
  digitalWrite(LEDBackward, LOW);   //backwards
  digitalWrite(LEDRight, LOW);      //right
  digitalWrite(LEDLeft, LOW);       //left

  currentHeading = getHeading();  // Call the Function getHeading and save the results in currentHeading

  //Serial.println("");             //start on a new line and Print out the Compass heading
  Serial.print("Compass: ");
  Serial.print(currentHeading);
  Serial.print("\t");

  IRsensorValue = analogRead(IRsensorPin);    //Find out if there is anything in the way
  Serial.print("IR Reading: ");
  Serial.print(IRsensorValue);
  Serial.print("\t");


  //Determine if the robot should move ("1") or if it should stay still ("0")
  // First case, moving pin == 0. Robot does not move.  movingPin is set to 0V.

  if (digitalRead(movingPin) == 0)
  {
    // turn the motors off
    analogWrite(pwm_a, 0);  //set both motors to run at 0% duty cycle (off)
    analogWrite(pwm_b, 0);
    //wait
    Serial.println("motors off");
    delay(1000);
  }

  // This else case is where the robot should move.  "movingPin ==1"
  else
  {
    // Check IR Sensor. If it is too close to something take evasive maneuvers
    if (IRsensorValue > obstacleSeen)
    {

      //evasive maneuvers
      Serial.println("");
      Serial.println("Danger!!  Object detected!");

      // Put it in reverse
      backward();
      // Turn to the Left
      left();
      left();
      left();
      // Move forward
      forward();

      //end of the evasive maneuvers

    }

    //The robot input is okay to "move" and nothing is in the way
    else {
      //Case where the robot is pointing from 0-270 Deg
      if (currentHeading < 180)
      {
        //turn slightly right
        right();
      }

      //Case where the robot is pointing from 270-359 Deg
      if (currentHeading > 180)
      {
        //turn slightly left
        left();

      }
    }
  }
}




void forward()
{
  digitalWrite(LEDForward, HIGH);
  Serial.println("Forward");
  digitalWrite(dir_a, HIGH);
  digitalWrite(dir_b, HIGH);
  analogWrite(pwm_a, speedOfRightWheel);
  analogWrite(pwm_b, speedOfLeftWheel);
  delay(MoveSize * 4);
  digitalWrite(LEDForward, LOW);
}




void backward()
{
  digitalWrite(LEDBackward, HIGH);
  Serial.println("Backward");
  digitalWrite(dir_a, LOW);
  digitalWrite(dir_b, LOW);
  analogWrite(pwm_a, speedOfRightWheel);
  analogWrite(pwm_b, speedOfLeftWheel);
  delay(MoveSize * 7);
  digitalWrite(LEDBackward, LOW);
}


void left()
{
  digitalWrite(LEDLeft, HIGH);
  Serial.println("Left Turn");
  digitalWrite(dir_a, HIGH);
  digitalWrite(dir_b, HIGH);
  analogWrite(pwm_a, speedOfRightWheel);
  analogWrite(pwm_b, speedOfLeftWheel - 50);  //spin the left wheel slower so the robot turns left
  delay(MoveSize);
  digitalWrite(LEDLeft, LOW);
}

void right()
{
  digitalWrite(LEDRight, HIGH);
  Serial.println("Right Turn");
  digitalWrite(dir_a, HIGH);
  digitalWrite(dir_b, HIGH);
  analogWrite(pwm_a, speedOfRightWheel - 50); //spin the right wheel slower so the robot turns right
  analogWrite(pwm_b, speedOfLeftWheel);
  delay(MoveSize);
  digitalWrite(LEDRight, LOW);
}


