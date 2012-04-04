#include <SoftwareSerial.h> //Allows us to define any digital pin to work as a serial UART pin.
#include "TinyGPS.h"  //Helps interpret GPS Data, (Downloaded NOT prat of the Arduino Core) 
TinyGPS gps; //Create instance

void setup()
{
 Serial1.begin(57600); // Baud rate of our GPS
 Serial.begin(57600);
}

void loop(){
 
/* The code below is simple! Made posible by the TinyGPS library I found online. It has built in
functions that output what ever we want...ie. Latitude or Longitude. The code is made so that 
boolean newData tells if something is coming in and update the LAT and LON*/ 

 bool newData = true; //To check if new data is coming in!
// For one second we parse GPS data and report values
 for (unsigned long start = millis(); millis() - start < 1000;)
 {
   while (Serial1.available())
   {
     char c = Serial1.read();
     //Serial.write(c); // Helps to see GPS data flow
     if (gps.encode(c)) // Did a new valid sentence come in?
       newData = true; //
   }
 }

 if (newData)
 {
   float flat, flon;
   unsigned long age;
   gps.f_get_position(&flat, &flon, &age);
   /*Serial.print("\nLAT= ");
   Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
   Serial.print("\nLON= ");
   Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
   Serial.print("\nSAT= ");
   Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
   Serial.print("\nPREC= ");
   Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
   Serial.print("\nAGE= ");
   Serial.print(age);
   Serial.print("\nCOURSE=");*/
   Serial.print(gps.course());
   Serial.print(",");
   //Serial.print("\n");
 }

}



