/* ////////////////////////////////
This sketch finds the heading (degrees) of the transmitting beacon relative to
the receiver. For example, if the receiver is due west of the beacon, this sketch
will return a heading of 90 degrees.
This sketch receives a packet and stores the RSSI (signal strength) in the RSSIArray
with an index of the Heading (data). It does so 'Samples' amount of times. Then the data
is passed through a digital filter. If the RSSI was not evaluated for a specific heading,
then that data point is not evaluated. This prevents values that were not measured from
effecting the output of the digital filter.
Authored By: Adam St. Amand
//////////////////////////////////*/





#include <XBee.h>

XBee xbee = XBee();
#define arraySize  180    // Determines RSSIArray size; Allows received headings to overwrite old ones
#define filterSamples  9           // filterSamples should  be an odd number, no smaller than 3 (<<# sensitive, #<< insensitive)
Rx16Response rx16 = Rx16Response();

float RSSIArray[arraySize];      // array for holding raw RSSI values
int sensSmoothArray [filterSamples];   // holds past RSSI values for filtering
int rawData, smoothData;  // variables for sensor data
int resetRSSI = 20;    //The value that RSSI is reset to after each pass through filter
int Samples = 110;


void setup() {
  //Initialize serial communications at 9600 bps:
  Serial.begin(57600); 
  Serial1.begin(57600);
  xbee.setSerial(Serial1);
  Serial.println("setup");
  Reset();
}




void loop() {
  
  for(int i = 0;i<Samples;i++)  Serial.println("try to retreive"); Retrieve();   Serial.println("retreive did something");
  
  //Passes all received data through a digital filter.
  for(int i = 0;i<arraySize;i++){
    while (RSSIArray[i] == resetRSSI && i <arraySize) i++;        //Skips any RSSI values that were not received (==reset value).
    smoothData = digitalSmooth(RSSIArray[i], sensSmoothArray);
    RSSIArray[i] = smoothData;
  }

  //Process the data once more, print the result, and reset.
  int finalHeading = (ProcessData());
  Serial.println("final heading");
  Serial.println(finalHeading);
  Reset();
}






/////////////////////////////////////////////////////
////////////////Local Functions//////////////////////
/////////////////////////////////////////////////////





//Resets all RSSI values to the same value.

void Reset(){
  for (int i = 0; i< arraySize; i ++) RSSIArray[i] = resetRSSI;  
}



//Receives the transmitted packet and stores the information in RSSIArray.

void Retrieve(){
  xbee.readPacket(50);    //Wait 50 to receive packet
  if (xbee.getResponse().isAvailable())     //Execute only if packet found
  {
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE) 
    {
      xbee.getResponse().getRx16Response(rx16);
      
      //Store the transmitted data and RSSI
      int currentHeading = rx16.getData(0);
      int currentRSSI = abs(rx16.getRssi()-100);

      //Stores the RSSI in RSSIArray. Only executes if the data is within parameters.
      if (currentHeading>=0 && currentHeading<=179){
        RSSIArray[currentHeading] = currentRSSI;
      }      
    }
  }
}






/* digitalSmooth
 Paul Badger 2007
 A digital smoothing filter for smoothing sensor jitter 
 This filter accepts one new piece of data each time through a loop, which the 
 filter inputs into a rolling array, replacing the oldest data with the latest reading.
 The array is then transferred to another array, and that array is sorted from low to high. 
 Then the highest and lowest %15 of samples are thrown out. The remaining data is averaged
 and the result is returned.
 Every sensor used with the digitalSmooth function needs to have its own array to hold 
 the raw sensor values. This array is then passed to the function, for it's use.
 This is done with the name of the array associated with the particular sensor.
 */
 
//Digital filter used to smooth the input data. 

int digitalSmooth(int rawIn, int *sensSmoothArray){     // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
  int j, k, temp, top, bottom;
  long total;
  static int i;
 // static int raw[filterSamples];
  static int sorted[filterSamples];
  boolean done;

  i = (i + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  sensSmoothArray[i] = rawIn;                 // input new data into the oldest slot

  // Serial.print("raw = ");

  for (j=0; j<filterSamples; j++){     // transfer data array into anther array for sorting and averaging
    sorted[j] = sensSmoothArray[j];
  }

  done = 0;                // flag to know when we're done sorting              
  while(done != 1){        // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (filterSamples - 1); j++){
      if (sorted[j] > sorted[j + 1]){     // numbers are out of order - swap
        temp = sorted[j + 1];
        sorted [j+1] =  sorted[j] ;
        sorted [j] = temp;
        done = 0;
      }
    }
  }

/*
  for (j = 0; j < (filterSamples); j++){    // print the array to debug
    Serial.print(sorted[j]); 
    Serial.print("   "); 
  }
  Serial.println();
*/

  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((filterSamples * 15)  / 100), 1); 
  top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j< top; j++){
    total += sorted[j];  // total remaining indices
    k++; 
    // Serial.print(sorted[j]); 
    // Serial.print("   "); 
  }

//  Serial.println();
//  Serial.print("average = ");
//  Serial.println(total/k);
  return total / k;    // divide by number of samples
}


          


//Finds the heading with maximum RSSI value and averages it
//with any headings that are within 1 RSSI unit. (>=maxRSSI-1)

int ProcessData(){
  int maxRSSI = 0, maxIndex = 0, counter = 0;
  maxRSSI = RSSIArray[maxIndex];
  
  //Find max RSSI value
  for (int i=1; i<arraySize; i++) {
    if (maxRSSI<RSSIArray[i]) maxRSSI = RSSIArray[i];
  }
  
  //Add all headings with RSSI>=maxRSSI - 1
  for (int i=0; i<arraySize; i++){
    if ((maxRSSI-1)<=RSSIArray[i])
    {
      maxIndex += i;
      counter++;
    }
  }
  
  return maxIndex/counter;    //Return the average of all headings >=maxRSSI - 1
}
