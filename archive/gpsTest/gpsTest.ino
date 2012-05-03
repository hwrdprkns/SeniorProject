#include <SoftwareSerial.h> //Allows us to define any digital pin to work as a serial UART pin.
#include "TinyGPS.h"  //Helps interpret GPS Data, (Downloaded NOT prat of the Arduino Core) 
#include "Streaming.h"

TinyGPS gps; //Create instance

#define GPSsrl Serial3

void setup()
{
 GPSsrl.begin(57600); // Baud rate of our GPS
 Serial.begin(9600);
 Serial << "Starting" <<endl;
}

void loop(){
 
/* The code below is simple! Made posible by the TinyGPS library I found online. It has built in
functions that output what ever we want...ie. Latitude or Longitude. The code is made so that 
boolean newData tells if something is coming in and update the LAT and LON*/ 

 bool newData = true; //To check if new data is coming in!
// For one second we parse GPS data and report values
 for (unsigned long start = millis(); millis() - start < 1000;)
 {
   while (GPSsrl.available())
   {
     char c = GPSsrl.read();
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
   Serial <<_FLOAT(flat,9) << "," <<_FLOAT(flon,9) << "," << age << "," << _FLOAT(gps.f_course(),4) << endl;
   
  }

}



