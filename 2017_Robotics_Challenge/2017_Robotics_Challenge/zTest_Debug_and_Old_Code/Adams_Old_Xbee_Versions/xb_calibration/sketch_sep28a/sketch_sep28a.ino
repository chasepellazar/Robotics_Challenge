/*This program will allow programs such as XCTU
communicate with the Xbee while attached to the fio v3
*/
void setup() {
  /*Start serial communication rates. Serial. for the arduino
  and Serial1. for the Xbee, The rates must be identical*/
  Serial.begin(57600); 
  Serial1.begin(57600);
}
void loop() // run over and over
{
  if (Serial1.available())
    Serial.write(Serial1.read());
  if (Serial.available())
    Serial1.write(Serial.read());
}
