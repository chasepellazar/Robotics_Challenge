
int pwm_a = 3;
int pwm_b = 11;
int dir_a = 12;
int dir_b = 13;

void setup() {
  // put your setup code here, to run once:
  pinMode(pwm_a, OUTPUT);
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly: 
  Serial.println("Preparing to move forward");

  //Move it forward
  digitalWrite(dir_a, HIGH);
  digitalWrite(dir_b, HIGH);

  //Set both motors to run at 100%
  analogWrite(pwm_a, 255);
  analogWrite(pwm_b, 255);
  Serial.println("Moving forward at 100%");
  delay(4000);

  //Set both motors to run at 50%
  analogWrite(pwm_a, 127);
  analogWrite(pwm_b, 127);
  Serial.println("Moving forward at 50%");
  delay(4000);


}





