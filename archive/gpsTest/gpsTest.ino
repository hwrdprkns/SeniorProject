#include <SoftwareSerial.h> //Allows us to define any digital pin to work as a serial UART pin.
#include "TinyGPS.h"  //Helps interpret GPS Data, (Downloaded NOT prat of the Arduino Core) 
#include "Streaming.h"

TinyGPS gps; //Create instance

#define GPSsrl Serial3
//testing coordinates
float LATITUDES[] = {42.40818023};
float LONGITUDES[] = { -71.11601257};


void setup()
{
 GPSsrl.begin(57600); // Baud rate of our GPS
 Serial.begin(57600);
 Serial << "Starting" <<endl;
}

void loop(){
 
/* The code below is simple! Made posible by the TinyGPS library I found online. It has built in
functions that output what ever we want...ie. Latitude or Longitude. The code is made so that 
boolean newData tells if something is coming in and update the LAT and LON*/ 

 bool newData = false; //To check if new data is coming in!
// For one second we parse GPS data and report values
 for (unsigned long start = millis(); millis() - start < 500;)
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
   Serial <<_FLOAT(flat,8) << "," <<_FLOAT(flon,8) << "," << age << "," << _FLOAT(gps.f_course(),4) << endl;
/*
   Serial.print("\nLAT= ");
   Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 8);
   Serial.print("\nLON= ");
   Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 8);
   Serial.print("\nSAT= ");
   Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
   Serial.print("\nPREC= ");
   Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
   Serial.print("\nAGE= ");
   Serial.print(age);
   Serial.print("\nCOURSE=");
   Serial.print(gps.course());
   Serial.print(",");
   Serial.print("\nf_course=");
   Serial.print(gps.f_course());
   Serial.print("\n");
   */
   int i = 0;
   double currentDistance = gps.distance_between(flat,flon,LATITUDES[i],LONGITUDES[i]);
 }

}



