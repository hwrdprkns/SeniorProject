#include "WayPoint.h"
#include "TinyGps.h"
#include <math.h>
#include "Streaming.h"
#include "Command.h"

#define GPSsrl Serial3

float LATITUDES[] = {42.408275,42.408024};
float LONGITUDES[] = { -71.115926, -71.116168};
int NUMBER_OF_WAYPOINTS = 2;

TinyGPS gps;
Command com;

float currentDistance;
float lastLatitude;
float lastLongitude;
unsigned long lastAge;

int state;

/**** For the command library ****/
int debug = 1;
extern ring_buffer rx_buf;
extern resultint_ resultint;
String atcmd = "";

void setup()
{
  GPSsrl.begin(57600); // Baud rate of our GPS
  
  state = 0; //default state
}

void loop()
{
  
  switch (state) {
	// sanity check
	case 0:
		checkSanity();
		state = 5;
		break;
  
	// initialization connection
	case 1:
		com.start_wifi_connection();
		com.drone_is_init = com.init_drone();
		state = 2;
		break;
	
	// takeoff state
	case 2:
		com.drone_takeoff();
		state = 5;
		break;
		
	// landing
	case 3:
		doShutdown();
		break;
		
	// emergency toggle
	case 4:
		com.drone_emergency_toggle();
		break;
		
	// flying state
	case 5:
        {
		double distance = WayPoint::computeDistance(getCurrent(1),getCurrent(0),LATITUDES[NUMBER_OF_WAYPOINTS-1],LONGITUDES[NUMBER_OF_WAYPOINTS-1]);  
        int timeSinceLastEncode = 0;
				
	   	if(readGPSData() && continueIfProperAge())navigatePath(0,distance);
		 
		break;
        }
		
//	//stop running
	default:
		while (1) {}
                break;
  }
} 

boolean readGPSData(){
	boolean fullSentence = false;
	while(GPSss.available()){
		int c = GPSss.read();
		fullSentence = gps.encode(c);
		if (fullSentence == true) return true;
	}
	return false;
}

long lastAge = 0;
/* This will return false if the age is invalid or too long. */
boolean continueIfProperAge(){
	
	if(lastAge == getAge()){
		int iter = 0;
		for(int iter = 0;iter<100;iter++){
			delay(5);
			readGPSData();
			if(lastAge != getAge()){
				lastAge = getAge();
				return true;
			}
		}
	}
	
	lastAge = getAge();
	
	return false;
}



void navigatePath(int state, double previousDistance){
  
  float destinationLat = LATITUDES[state];
  float destinationLong = LONGITUDES[state];
  
  float[] currentLocation = getCurrent();
  
  int flightStatus = fly_to(currentLocation[0],currentLocation[1],destinationLat,destinationLong); //Maybe return some kind of flight status here?
  
  if(!(currentDistance < previousDistance)) emergencySituation(-1);//Need to handle if we get no closer.
  
  
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
  

float[] getCurrent(int param){
	
	float latitude,longitude;
	unsigned long age;
	
	gps.f_get_position(&latitude,&longitude,&age);

       lastLatitude = latitude;
       lastLongitude = longitude;
       lastAge = age;
	
	return {latitude,longitude};
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








