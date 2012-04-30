#include "TinyGps.h"
#include "Streaming.h"
#include "Command.h"
#include <math.h>

#define GPSss Serial3

float LATITUDES[] = {42.40859985,42.40861892};
float LONGITUDES[] = { -71.11579895, -71.11585235};

int NUMBER_OF_WAYPOINTS = 2;

TinyGPS gps;
Command com;

float currentDistance;
int locationStep;
int state;
unsigned long lastAge;

struct Location {
  float latitude;
  float longitude;
  unsigned long age;
} currentLocation;

/**** For the command library ****/
int debug = 1;
extern ring_buffer rx_buf;
extern resultint_ resultint;
String atcmd = "";

void setup()
{
  GPSss.begin(57600); // Baud rate of our GPS
  PCsrl.begin(9600);
  
  state = 0; //default state
  locationStep = 0; //initalize location step
}

void loop()
{
  
  switch (state) {
	// sanity check
	case 0:
        while(!readGPSData() && !checkSanity()){delay(500);}
		state = 1;
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
	if(readGPSData() && continueIfProperAge())navigatePath();	 
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
		if (fullSentence == true) {
                  getCurrent();
                  return true;
                }
	}

        PCsrl << "readGPSData retuned false" << endl;
	return false;
}

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
	PCsrl << "Did not return a proper age." << endl;
	return false;
}


void navigatePath(){
  
	float destinationLat;
	float destinationLong;
  
	double currentDistance;
	
	for (int i = 0; i < NUMBER_OF_WAYPOINTS; i++ ) {
		bool done = false;
		destinationLat = LATITUDES[i];
		destinationLong = LONGITUDES[i];

	while(!done)
	{
		if(readGPSData() && continueIfProperAge()) {
			fly_to(currentLocation.latitude,currentLocation.longitude,destinationLat,destinationLong); //Maybe return some kind of flight status here?
		}
		currentDistance = TinyGPS::distance_between(currentLocation.latitude,currentLocation.longitude,LATITUDES[i],LONGITUDES[i]);
		if(abs(currentDistance) < 6) done = true;
	}
	}
	state = 3;
 
}

int fly_to(float startLat,float startLong,float endLat,float endLon){
	
	float bearing = getCourse(startLat,startLong,endLat,endLon);
	
	//PCsrl.print("Calculated bearing:");PCsrl.println(bearing);
	
	float distance = TinyGPS::distance_between(startLat,startLong,endLat,endLon);
	currentDistance = distance;

    //PCsrl.print("Calculated distance:");PCsrl.println(distance);
        
	com.moveRotate(ceil(bearing));
	com.moveForward(ceil(distance/5));
    com.drone_hover(2500);
        
}

float getCourse(float startLat,float startLong,float endLat,float endLon){
  // First get the course that we need to be. 
  
  float courseOnCourse = TinyGPS::course_to(startLat,startLong,endLat,endLon);
  float myCourse = gps.f_course();
  
  return (courseOnCourse-myCourse) < 0 ? (courseOnCourse-myCourse):(courseOnCourse-myCourse+360); 
}

void getCurrent(){
      gps.f_get_position(&currentLocation.latitude,&currentLocation.longitude,&currentLocation.age);
}

unsigned long getAge(){
   gps.f_get_position(&currentLocation.latitude,&currentLocation.longitude,&currentLocation.age);
  return currentLocation.age;
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

  double distanceSanity = TinyGPS::distance_between(LATITUDES[0],LONGITUDES[0],LATITUDES[NUMBER_OF_WAYPOINTS-1],LONGITUDES[NUMBER_OF_WAYPOINTS-1]);

  boolean isSaneDistance = distanceSanity < 1000;

  return isSaneDistance;
}








