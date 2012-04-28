#include "WayPoint.h"
#include "TinyGps.h"
#include <math.h>
#include <SoftwareSerial.h>
#include "Streaming.h"
#include "Command.h"

#define GPSsrl Serial

float LATITUDES[] = {42.408275,42.408024};
float LONGITUDES[] = { -71.115926, -71.116168};
int NUMBER_OF_WAYPOINTS = 2;

TinyGPS gps;
Command com;

float currentDistance;
float lastLatitude;
float lastLongitude;
unsigned long lastAge;

/**** For the command library ****/
int debug = 1;
extern ring_buffer rx_buf;
extern resultint_ resultint;
int sequenceNumber = 1;
String atcmd = "";

void setup()
{
  GPSsrl.begin(57600); // Baud rate of our GPS
  
  com.start_wifi_connection();
  com.drone_is_init = com.init_drone();
  
}

void loop()
{
  if (com.drone_is_init == 0 && debug == true) {
      PCsrl << "Drone wasn't initlized before loop() was called. Initalizing now.\r\n";
   }
  
  checkSanity();
  
  double distance = WayPoint::computeDistance(getCurrent(1),getCurrent(0),LATITUDES[NUMBER_OF_WAYPOINTS-1],LONGITUDES[NUMBER_OF_WAYPOINTS-1]);  
  navigatePath(0,distance);
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
	
	float bearing = getCourse(startLat,startLong,endLat,endLon);
	
	PCsrl.print("Calculated bearing:");PCsrl.println(bearing);
	
	float distance = TinyGPS::distance_between(startLat,startLong,endLat,endLon);
	currentDistance = distance;

        PCsrl.print("Calculated distance:");PCsrl.println(distance);
	
	com.moveForward(distance);
}

float getCourse(float startLat,float startLong,float endLat,float endLon){
  // First get the course that we need to be. 
  
  float courseOnCourse = TinyGPS::course_to(startLat,startLong,endLat,endLon);
  float myCourse = gps.f_course();
  
  return (courseOnCourse-myCourse) < 0 ? (courseOnCourse-myCourse):(courseOnCourse-myCourse+360);
  
  
}
  

float getCurrent(int param){
	
	float latitude,longitude;
	unsigned long age;
	
	gps.f_get_position(&latitude,&longitude,&age);

       lastLatitude = latitude;
       lastLongitude = longitude;
       lastAge = age;
	
	if(param == 1) return latitude;
	
	return longitude;
}

unsigned long getAge(){
  float latitude,longitude;
  unsigned long age;
  gps.f_get_position(&latitude,&longitude,&age);
  
  lastLatitude = latitude;
  lastLongitude = longitude;
  lastAge = age;
  
  return age;
}

void emergencySituation(int emergency){
	
	if(emergency == -1){
		doShutdown();
	}
	
}

void doShutdown(){
  
  com.drone_landing();
	
} 
  
boolean checkSanity(){

  double distanceSanity = WayPoint::computeDistance(LATITUDES[0],LONGITUDES[0],LATITUDES[NUMBER_OF_WAYPOINTS-1],LONGITUDES[NUMBER_OF_WAYPOINTS-1]);
  double bearingSanity = WayPoint::computeInitialBearing(LATITUDES[0],LONGITUDES[0],LATITUDES[NUMBER_OF_WAYPOINTS-1],LONGITUDES[NUMBER_OF_WAYPOINTS-1]);
  double finalSanity = WayPoint::computeFinalBearing(LATITUDES[0],LONGITUDES[0],LATITUDES[NUMBER_OF_WAYPOINTS-1],LONGITUDES[NUMBER_OF_WAYPOINTS-1]);

  boolean isSaneDistance = distanceSanity < 1000;

  return isSaneDistance;
}








