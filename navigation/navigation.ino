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
  Serial.begin(57600); // Baud rate of our GPS

}

void loop()
{
  checkSanity();
  navigatePath(0,WayPoint::computeDistance(getCurrent(1),getCurrent(0),LATITUDES[NUMBER_OF_WAYPOINTS-1],LONGITUDES[NUMBER_OF_WAYPOINTS-1]));
} 

void navigatePath(int state, double previousDistance){
  
  float destinationLat = LATITUDES[state];
  float destinationLong = LONGITUDES[state];
  
  float currentLatitude = getCurrent(1);
  float currentLongitude = getCurrent(0);
  
  int flightStatus = fly_to(currentLatitude,currentLongitude,destinationLat,destinationLong); //Maybe return some kind of flight status here?
  
  if(!(currentDistance < previousDistance)) emergencySituation(-1);//Need to handle if we get no closer.
  
  if(lastAge == getAge()){
    int i =0;
    while(lastAge == getAge()){
      i++;
      if(i > 100){doShutdown();return;}
      //TODO Make Drone hover where it is. 
    }
  }
    
  if(currentDistance < 10){doShutdown(); return;}
    
  navigatePath(state + 1, currentDistance);
 
}

int fly_to(float startLat,float startLong,float endLat,float endLon){
	
	float bearing = (float) WayPoint::computeInitialBearing(startLat,startLong,endLat,endLon);
	
	//TODO: Send bearing command to Drone
	
	float distance = (float) WayPoint::computeDistance(startLat,startLong,endLat,endLon);
	currentDistance = distance;
	
	//TODO: Send distance command to Drone. 
}

float getCurrent(int param){
	
	float latitude,longitude;
	unsigned long age;
	
	gps.f_get_position(&latitude,&longitude,&age);
	
	if(param == 1) return latitude;
	
	return longitude;
}

unsigned long getAge(){
  float latitude,longitude;
  unsigned long age;
  gps.f_get_position(&latitude,&longitude,&age);
  return age;
}

void emergencySituation(int emergency){
	
	if(emergency == -1){
		doShutdown();
	}
	
}

void doShutdown(){
  
  //Send shutdown command to Drone. 
	
} 
  
boolean checkSanity(){

  double distanceSanity = WayPoint::computeDistance(LATITUDES[0],LONGITUDES[0],LATITUDES[NUMBER_OF_WAYPOINTS-1],LONGITUDES[NUMBER_OF_WAYPOINTS-1]);
  double bearingSanity = WayPoint::computeInitialBearing(LATITUDES[0],LONGITUDES[0],LATITUDES[NUMBER_OF_WAYPOINTS-1],LONGITUDES[NUMBER_OF_WAYPOINTS-1]);
  double finalSanity = WayPoint::computeFinalBearing(LATITUDES[0],LONGITUDES[0],LATITUDES[NUMBER_OF_WAYPOINTS-1],LONGITUDES[NUMBER_OF_WAYPOINTS-1]);

  WayPoint::printDouble(distanceSanity,5);
  WayPoint::printDouble(bearingSanity,5);
  WayPoint::printDouble(finalSanity,5);

  //boolean droneSanity = checkDroneSanity();

  boolean isSaneDistance = distanceSanity < 1000;

  return isSaneDistance;
}







