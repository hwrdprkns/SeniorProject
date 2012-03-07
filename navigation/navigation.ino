#include "WayPoint.h"
#include <math.h>

/**

This is the sketch for the following functions:

SantiyCheck
calculateWaypoints
**/

double LATITUDES[] = {42.408083,42.40798,42.407934};
double LONGITUDES[] = {-71.116326,-71.116253, -71.115977};
int NUMBER_OF_WAYPOINTS = 3;

void setup()
{
  Serial.begin(9600);
}



void loop()
{
  checkSanity();
}

boolean checkSanity(){
  
  double distanceSanity = WayPoint::computeDistanceAndBearing(LATITUDES[0],LONGITUDES[0],LATITUDES[NUMBER_OF_WAYPOINTS-1],LONGITUDES[NUMBER_OF_WAYPOINTS-1]);
  double bearingSanity = WayPoint::computeInitialBearing(LATITUDES[0],LONGITUDES[0],LATITUDES[NUMBER_OF_WAYPOINTS-1],LONGITUDES[NUMBER_OF_WAYPOINTS-1]);
  double finalSanity = WayPoint::computeFinalBearing(LATITUDES[0],LONGITUDES[0],LATITUDES[NUMBER_OF_WAYPOINTS-1],LONGITUDES[NUMBER_OF_WAYPOINTS-1]);
 
  printDouble(distanceSanity,5);
  printDouble(bearingSanity,5);
  printDouble(finalSanity,5);
  
  Serial.write("\n");
  
  //boolean droneSanity = checkDroneSanity();
  
  return (distanceSanity < 1000);
}



 
 
 
 void printDouble(double val, byte precision){
  // prints val with number of decimal places determine by precision
  // precision is a number from 0 to 6 indicating the desired decimial places

  if(val < 0.0){
    Serial.print('-');
    val = -val;
  }

  Serial.print (int(val));  //prints the int part
  if( precision > 0) {
    Serial.print("."); // print the decimal point
    unsigned long frac;
    unsigned long mult = 1;
    byte padding = precision -1;
    while(precision--)
  mult *=10;

    if(val >= 0)
 frac = (val - int(val)) * mult;
    else
 frac = (int(val)- val ) * mult;
    unsigned long frac1 = frac;
    while( frac1 /= 10 )
 padding--;
    while(  padding--)
 Serial.print("0");
    Serial.print(frac,DEC) ;
  }
  Serial.write("\n");
}
 




