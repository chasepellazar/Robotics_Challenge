/*
This sketch simply receives a packet and prints the packet's
data and signal strength(RSSI). 
It is highly reccomended you upload this sketch and ensure
you are receiving the data you expect(the correct headings) BEFORE
conducting any tests/mesurements with the Xbee/fio. This is often
the fastest way to find out if anything is interfering with the Xbee
or if something is wrong with the transmission.  
*/


#include <printers.h>
#include <XBee.h>

XBee xbee = XBee();
Rx16Response rx16 = Rx16Response();

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(57600); 
  Serial1.begin(57600);
  xbee.setSerial(Serial1);
}

void loop() {
 
  xbee.readPacket(10);    // Waits 50ms to read a packet
  if (xbee.getResponse().isAvailable())   //Executes only if a packet is available to be read
  {
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE) xbee.getResponse().getRx16Response(rx16);
    
    //Prints Heading
    Serial.print("Heading = ");
    Serial.println(rx16.getData(0));
    
    //Prints signal strength(RSSI)
    Serial.print("RSSI = ");
    Serial.println(abs(rx16.getRssi()-100));
  }
}



