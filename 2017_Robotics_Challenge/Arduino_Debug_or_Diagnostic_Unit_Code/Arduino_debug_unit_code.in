/*
  This code writes from the serial in to an LCD.
  Designed for debug unit of RF Beacon system for COSGC
  Robert Belter 10/30/2015
 */

String toDisplay = "RF Beacon Debug Unit";
//If using a 16x2 LCD, uncommment the below line
//toDisplay = "Debug Unit";

#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup() {
  // Set up columuns and rows. Note: We used a 20x4 LCD, 16x2 is more common
  lcd.begin(20, 4);
  
  // Print a message to the LCD.
  lcd.print(toDisplay);

  delay(1000);
  Serial.begin(57600);
}

void loop() {
  // set the cursor to column 0, line 1
  lcd.setCursor(0, 1);
  if(Serial.available()){
    lcd.clear();
    lcd.print(toDisplay);
  }
  while(Serial.available()){
    lcd.write(Serial.read());
  }
}

