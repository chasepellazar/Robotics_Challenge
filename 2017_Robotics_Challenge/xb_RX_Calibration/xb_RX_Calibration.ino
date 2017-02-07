/*
 * This Sketch will output the axis offsets sent by xbee TX
 * code if ouput_calibration flag is set
 * Authored by Robert Belter 10/30/2015
 */
#include <XBee.h>

XBee xbee = XBee();
Rx16Response rx16 = Rx16Response();

union{
  int i[3];
  uint8_t b[6];
}calibration_converter;

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
      for(int i = 0; i<6; i++) calibration_converter.b[i] = rx16.getData(i);

      //Print to console
      char buffer[100];
      sprintf(buffer, "%d, %d, %d", calibration_converter.i[0],calibration_converter.i[1],calibration_converter.i[2]);
      Serial.println(buffer);
    }
  }
}
