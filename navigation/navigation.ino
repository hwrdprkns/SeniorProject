#include "WayPoint.h"
#include "TinyGps.h"
#include <math.h>
#include <SoftwareSerial.h>

/**
 * This is the sketch for the following functions:
 * 
 * SantiyCheck
 * calculateWaypoints
 **/

float LATITUDES[] = {42.408083,42.40798,42.407934};
float LONGITUDES[] = {-71.116326,-71.116253, -71.115977};
int NUMBER_OF_WAYPOINTS = 3;

TinyGPS gps;

float currentDistance;

void setup()
{
  Serial1.begin(57600); // Baud rate of our GPS
  Serial.begin(57600);
}
 


void loop()
{
  checkSanity();
  //navigatePath(0,WayPoint::calculateDistance(getLatitudeFromGPS(),getLongitudeFromGPS(),LATITUDES[NUMBER_OF_WAYPOINTS-1],LONGITUDES[NUMBER_OF_WAYPOINTS-1]);
  return;
} 

void navigatePath(int state, double previousDistance){
  
  double destinationLat = LATITUDES[state];
  double destinationLong = LONGITUDES[state];
  
  int flightStatus = fly_to(getCurrent(1),getCurrent(0),destinationLat,destinationLong); //Maybe return some kind of flight status here?
  
  if(!(currentDistance < previousDistance)) emergencySituation(-1);//Need to handle if we get no closer.
  
  if(currentDistance < 10){doShutdown(); return;}
    
  navigatePath(state + 1, currentDistance);
 
}

int fly_to(float startLat,float startLong,float endLat,float endLon){
	
	float bearing = (float) WayPoint::computeInitialBearing(startLat,startLong,endLat,endLon);
	
	//Send bearing command to Drone
	
	float distance = (float) WayPoint::computeDistance(startLat,startLong,endLat,endLon);
	currentDistance = distance;
	
	//Send distance command to Drone. 
}

float getCurrent(int param){
	
	float latitude,longitude;
	unsigned long age;
	
	gps.f_get_position(&latitude,&longitude,&age);
	
	if(param == 1) return latitude;
	
	return longitude;
}

void emergencySituation(int emergency){
	
	if(emergency == -1){
		doShutdown();
	}
	
}

void doShutdown(){
	
}
  
  
boolean checkSanity(){

  double distanceSanity = WayPoint::computeDistance(LATITUDES[0],LONGITUDES[0],LATITUDES[NUMBER_OF_WAYPOINTS-1],LONGITUDES[NUMBER_OF_WAYPOINTS-1]);
  double bearingSanity = WayPoint::computeInitialBearing(LATITUDES[0],LONGITUDES[0],LATITUDES[NUMBER_OF_WAYPOINTS-1],LONGITUDES[NUMBER_OF_WAYPOINTS-1]);
  double finalSanity = WayPoint::computeFinalBearing(LATITUDES[0],LONGITUDES[0],LATITUDES[NUMBER_OF_WAYPOINTS-1],LONGITUDES[NUMBER_OF_WAYPOINTS-1]);

  printDouble(distanceSanity,5);
  printDouble(bearingSanity,5);
  printDouble(finalSanity,5);

  Serial.write("\n");

  //boolean droneSanity = checkDroneSanity();

  boolean isSaneDistance = distanceSanity < 1000;

  if(isSaneDistance)
    Serial.write("This is a sane distance \n");
  else
    Serial.write("The distance is too long \n");

  return isSaneDistance;
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






