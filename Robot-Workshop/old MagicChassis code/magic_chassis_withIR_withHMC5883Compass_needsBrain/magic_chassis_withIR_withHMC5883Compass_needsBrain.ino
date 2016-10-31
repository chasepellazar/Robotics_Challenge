/*
 
 Title:  magic_chassis_withIR_withHMC5883Compass_needsBrain
 
 Description: 
 Using the Magician Chassis, with Arduino Moto driver shied, IR sensor and HMC5883L Compass.  Detect obstacles and back away.
 
 Version 1.0  10/20/14
 */

#include <Wire.h> 
#include <HMC5883L.h>

HMC5883L compass;  // Storing the features in the HMC5883 library as a variable called compass

//Defining Arduino Pin Assignments

//A is the Right Wheel and B is the Left Wheel
int pwm_a = 3;   // Right Wheel Speed
int pwm_b = 11;  // Left Wheel Speed
int dir_a = 12;  // Right Wheel Direction
int dir_b = 13;  // Left Wheel Direction


int IRsensorPin = A0;    // Define the input pin for the IR sensor
int IRsensorValue = 0;   // Variable to store the Integer value coming from the IR sensor
int LEDPin = 9 ;         // On when in IR sensor is triggered and is backing up
int movingPin = 8;       //Enable movement of the robot.  Low for sensor/Serial work, High moving around (Movement Enabled)


float currentHeading = 0.0;  // Variable to hold Heading in degrees

void setup()
{
  Serial.begin(9600);
  Wire.begin();                // join i2c bus

  pinMode(pwm_a, OUTPUT);      //Set motor control pins to be outputs from the Arduino
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);

  pinMode(IRsensorPin, INPUT);   // Reading from the IR sensor
  pinMode(LEDPin, OUTPUT);       // Output pin to show state information such as needing to back up
  pinMode(movingPin, INPUT);     // Enables movement but still collects compass and IR readings and reports to serial 

  analogWrite(pwm_a, 0);          //set both motors speed to 0% duty cycle (off)
  analogWrite(pwm_b, 0);

  compass = HMC5883L();           // new instance of HMC5883L compass
  setupHMC5883L();                // setup the HMC5883L
}

void loop()
{

  digitalWrite(LEDPin, LOW);      // Turn the LED off 

  currentHeading = getHeading();  // Call the Function getHeading and save the results in currentHeading

    Serial.println("");             //start on a new line and Print out the Compass heading
  Serial.print("Compass: ");
  Serial.print(currentHeading);
  Serial.print("\t");


  IRsensorValue = analogRead(IRsensorPin);    //Find out if there is anything in the way 
  Serial.print("IR Reading: ");
  Serial.print(IRsensorValue);
  Serial.print("\t");

  //Determine if the robot should move ("1") or if it should stay still ("0")
  if(digitalRead(movingPin) ==0)
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
    // Check IR Sensor, if it is too close to something take evasive maneuvers
    if(IRsensorValue > 500)
    {
      //Show that the IR Sensor determined something is in the way
      digitalWrite(LEDPin, HIGH);
      //evasive maneuvers
      Serial.println("");
      Serial.println("Danger!!  Backup Quick!");

      //Put it in reverse
      digitalWrite(dir_a, LOW);  
      digitalWrite(dir_b, LOW);  

      analogWrite(pwm_a, 255);  //set both motors to run at 100% duty cycle (fast)
      analogWrite(pwm_b, 255);
      delay(2000);

      //turn a bit to the Left
      Serial.println("Tank Track pivot to the Left");
      digitalWrite(dir_a, HIGH);  // Right motor forward
      digitalWrite(dir_b, LOW);   // Left motor backward

      analogWrite(pwm_a, 255);  //set both motors to run at 100% duty cycle (fast)
      analogWrite(pwm_b, 255);
      delay(1000);
      //end of the evasive maneuvers

    }

    //The robot input is set to "move" and nothing is in the way
    else{
      //Case where the robot is pointing from 0-270 Deg
      if(currentHeading < 270)      
      {
        //turn right
        Serial.print("Turning right");

        //Something needed here

      }

      //Case where the robot is pointing from 270-359 Deg
      if (currentHeading > 270)
      {
        //turn left
        Serial.print("Turning left");

        // Something needed here
      }
      delay(100);  //move in that direction for 100ms  
    }
  }
}












