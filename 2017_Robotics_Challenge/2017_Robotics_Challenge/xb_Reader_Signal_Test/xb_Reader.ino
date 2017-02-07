/*
 * This Sketch will output a heading recieved via xbee, sent as a float
 * Along side the signal strength
 * Authored by Robert Belter 10/30/2015
 */
#include <XBee.h>

XBee xbee = XBee();
Rx16Response rx16 = Rx16Response();

union{
  float f;
  uint8_t b[4];
}heading_converter;

void setup() {
  //Initialize serial communications at 57600 bps:
  Serial.begin(57600); 
  Serial1.begin(57600);
  xbee.setSerial(Serial1);
}

void loop() {
  xbee.readPacket();
  if (xbee.getResponse().isAvailable())
  {
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE) 
    {
      xbee.getResponse().getRx16Response(rx16);
      
      //Store the transmitted data and RSSI
      for(int i = 0; i<4; i++) heading_converter.b[i] = rx16.getData(i);
      int currentRSSI = -rx16.getRssi();

      //Print to console
      Serial.print("Recieved: ");
      Serial.print(heading_converter.f, 2);
      Serial.print(" with strength ");
      Serial.print(currentRSSI);
      Serial.println("dB");
    }
  }
}
