/*  
 
 Description:
 Using an input on A0, 
 LEDS should be hooked up to Pins 4, 5, 6 and 7. 
 
 COSGC Robotics Workshop
 Oct 10 2013  1.0  Initial Verison
 Jun 10 2015  2.0  Update pin asignment to work with other workshop segements
 
 */

int sensorValue = 0;
const int analogInPin = A0;

void setup()  {
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(analogInPin, INPUT);
  Serial.begin(9600);
}

void loop() {

  //Get the input reading
  sensorValue = analogRead(analogInPin);
  Serial.print("sensorValue = ");
  Serial.println(sensorValue);

  //Turn all LEDs OFF to start

  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);

  delay(sensorValue);
  digitalWrite(4, HIGH);
  delay(sensorValue);
  digitalWrite(5, HIGH);
  delay(sensorValue);
  digitalWrite(6, HIGH);
  delay(sensorValue);
  digitalWrite(7, HIGH);
  delay(sensorValue);

}





