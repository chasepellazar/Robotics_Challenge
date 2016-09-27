/*This program will allow programs such as XCTU
communicate with the Xbee while attached to the fio v3
*/
void setup() {
  /*Start serial communication rates. Serial. for the arduino
  and Serial1. for the Xbee, The rates must be identical
  **If first time configuring Xbee, use the 9600 code. 
  After configuration of the Xbee through XCTU, 
  reupload code with the 9600 commented out and 
  56700 uncommented.*/
  
  //First Use Baud
  //Serial.begin(9600);
  //Serial1.begin(9600);

  //Use After Xbee is configured to 57600 Baud
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
