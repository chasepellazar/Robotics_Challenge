/*  
 
 Description:
 Using an input on A0, read the value and turn on LEDS in greater succession based on that value.
 
 LEDS should be hooked up to Pins 4, 5, 6 and 7. 
 
 COSGC Robotics Workshop
 Oct 10 2013  1.0  Initial Verison
 Jun 10 2015  2.0  Update pin asignment to work with other workshop segements
 
 */

int sensorValue = 0;
float sensorVoltage = 0.0;
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
  Serial.print(sensorValue);

  //Convert into Volts
  sensorVoltage= sensorValue * (5.0 / 1023.0);
  Serial.print("\t sensorVoltage = ");
  Serial.println(sensorVoltage);  


  //Turn all LEDs OFF to start
  //Then Turn LEDs ON if the input value is met
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);

  //The First LED
  if(sensorVoltage>1.0) {
    digitalWrite(4, HIGH);
  }

  //The Second LED
  if(sensorVoltage>1.5) {
    digitalWrite(5, HIGH);
  }

  //The Third LED
  if(sensorVoltage>2.5)  {
    digitalWrite(6, HIGH);
  }

  //The Fourth LED
  if(sensorVoltage>3.0)  {
    digitalWrite(7, HIGH);
  }
  delay(50);
}





