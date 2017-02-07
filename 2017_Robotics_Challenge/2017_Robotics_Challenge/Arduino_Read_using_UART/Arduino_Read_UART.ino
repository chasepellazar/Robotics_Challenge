/*
  Example Code to read output from reciever over UART
  Robert Belter 10/30/2015
 */

#include <SoftwareSerial.h>
//RX on pin 10
SoftwareSerial mySerial(10, 11);

char bufferRead[20];

void setup() {
  mySerial.begin(57600);
  Serial.begin(57600);
}

void loop() {
  if(mySerial.available()){
    Serial.println(getHeading());
  }
}

int getHeading(){
  int i = 0;
  delay(2);
  while(mySerial.available()){
    bufferRead[i] = mySerial.read();
    i++;
  }
  bufferRead[i] = '\0';
  String number = bufferRead;
  return number.toInt();
}

