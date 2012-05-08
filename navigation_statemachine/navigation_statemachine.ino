#include "TinyGps.h"
#include "Streaming.h"
#include "Command.h"
#include <math.h>

#define GPSsrl Serial3

/* basketball court */
/*
float LATITUDES[] = {42.40859985,42.40861892};
float LONGITUDES[] = { -71.11579895, -71.11585235};
*/   

/* volleyball court */
/*
float LATITUDES[] = {42.40882873,42.40877914};
float LONGITUDES[] = { -71.11622619, -71.11621093};
*/

//volleyball court 2
//42.408779144,-71.116058349
/*
float LATITUDES[] = {42.408779144};
float LONGITUDES[] = { -71.116058349};
*/
//acquired by verifypropergps, in front of halligan
//current point 0 lat: 42.40818023 log: -71.11601257
float LATITUDES[] = {42.40818023};
float LONGITUDES[] = { -71.11601257};

enum GPSSTATUS {NOSIG, NOCOURSE, GOOD};
int NUMBER_OF_WAYPOINTS = 1;

TinyGPS gps;
Command com;

int locationStep;
int state;
unsigned long lastAge;
unsigned long lastCourse; // used for check validity of GPS data

struct Location {
  float latitude;
  float longitude;
  unsigned long age;
} currentLocation;

/**** For the command library ****/
int debug = 1;
String atcmd = "";

void setup()
{
  GPSsrl.begin(57600); // Baud rate of our GPS
  PCsrl.begin(57600);
  
  state = 0; //default state
  locationStep = 0; //initalize location step
}

void loop()
{
  
  switch (state) {
	// sanity check
	case 0:
		/* only quit while loop when proper GPS signal is acquired */
        while( ( verifyPropergps() != NOSIG ) && !checkSanity()){delay(100);}
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
		com.drone_hover(1000);
		com.moveForward(0.5);  //move a small distance to enable bearing calculation on GPS
		com.drone_hover(1000);
		state = 5;
		break;
		
	// landing
	case 3:
		doShutdown();
		state = -1;
		break;
		
	// emergency toggle
	case 4:
		com.drone_emergency_toggle();
		break;
		
	// flying state
	case 5:
		navigatePath();
		//com.drone_hover(1000);
                //delay(1000);
		state = 3;
		break;
        
		
	//stop running
	default:
		while (1) {}
  }
} 

boolean readGPSData(){
	boolean fullSentence = false;
	while(GPSsrl.available()){
		int c = GPSsrl.read();
		fullSentence = gps.encode(c);
		if (fullSentence == true) {
                  return true;
                }
	}

        //PCsrl << "readGPSData returned false" << endl;
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

// proper way to gather GPS data and gaurantee its validity
// return false if no valid GPS data, vice versa
int verifyPropergps(){
	int i;
	// the gps updates its coordinates about 5 times per second, so we want to check it less frequently

	// better to loop without delays
	// read GPS data continuously for a maximum of 250 ms
	for (unsigned long time = millis(); (millis()-time) < 250;) {
		if (readGPSData()) break;
	}
	getCurrent();
	
	//if the data is longer than 0.5 sec
	if( currentLocation.age > 500 || currentLocation.age <= 0){
		return NOSIG;
	}
	if  ( lastCourse == gps.course() ) {
		return NOCOURSE;
        } 
	else lastCourse = gps.course();
	
	return GOOD;
}


void navigatePath(){
  
	float destinationLat; 
	float destinationLong; 
  
	double currentDistance;
    unsigned long gpstime,gpsdate,gpsage;
	for (int i = 0; i < NUMBER_OF_WAYPOINTS; i++ ) {
		bool done = false;
		destinationLat = LATITUDES[i];
		destinationLong = LONGITUDES[i];
		int hovercount = 0;

	while(!done)
	{        //PCsrl << "going to point " << i << " lat: " << _FLOAT(destinationLat,8) << " log: " << _FLOAT(destinationLong,8) <<endl;
		//if(verifyPropergps()) {
		int a = verifyPropergps();
		switch (a) {
		case GOOD: {
  		currentDistance = TinyGPS::distance_between(currentLocation.latitude,currentLocation.longitude,LATITUDES[i],LONGITUDES[i]);
		if(abs(currentDistance) < 5) {
                //PCsrl << "follow point " << i <<" success, current distance" << currentDistance <<endl;
                done = true;
	        }
                  	gps.get_datetime(&gpsdate,&gpstime,&gpsage);
                  PCsrl << "current gps time" << gpstime <<endl;
                  //PCsrl << "current point " << i << " lat: " << _FLOAT(currentLocation.latitude,8) << " log: " << _FLOAT(currentLocation.longitude,8) <<endl;
			hovercount = 0;
			fly_to(currentLocation.latitude,currentLocation.longitude,destinationLat,destinationLong); //Maybe return some kind of flight status here?
			break;
		}
		
		//else{ 
		case NOSIG: {
            if (debug) { PCsrl << "gps data acquiring failed" <<endl;}
			//com.drone_hover(200);
			hovercount++;
			// if hovercount reaches 20, means no valid GPS signal for 4 sec,
			// quit
			if ( hovercount > 20 ) {
				return;
			}
			break;
		}
		
		case NOCOURSE: {
			if (debug) { PCsrl << "gps data no course update" <<endl;}
			//com.moveForward(1);
			delay(200);
			break;
		}
		}
    }
	}
}

int fly_to(float startLat,float startLong,float endLat,float endLon){
	
	float bearing = getCourse(startLat,startLong,endLat,endLon);

	float distance = TinyGPS::distance_between(startLat,startLong,endLat,endLon);
      PCsrl.print("Calculated bearing:");PCsrl.print(bearing);
      PCsrl.print(" distance:");PCsrl.println(distance);     
	//com.moveRotate(ceil(bearing));
	// Ed: i fixed the moveForward code, now 1 meter actually means 1 meter (more or less)
	//com.moveForward(ceil(distance/5));
    //com.drone_hover(200);
    delay(1000);

        
	return 1;
}

float getCourse(float startLat,float startLong,float endLat,float endLon){
  // First get the course that we need to be. 
  
  float courseOnCourse = TinyGPS::course_to(startLat,startLong,endLat,endLon);
  float myCourse = gps.f_course();
  
  return (courseOnCourse-myCourse) >= 0 ? (courseOnCourse-myCourse):(courseOnCourse-myCourse+360); 
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








