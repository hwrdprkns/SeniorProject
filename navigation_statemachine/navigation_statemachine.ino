/* navigation_statemachine.ino
 * created by Team Vermillion, Millenium Falcon
 * cycle through a statemachine to navigate the drone through GPS coordinates
 * Last update: 5/11/2012
 */

#include "TinyGps.h"
#include "Streaming.h"
#include "Command.h"
#include <math.h>

#define GPSsrl Serial3

/* GPS coordinates specification
 * recode the latitudes and longitudes in corresponding arrays
 * modify the NUMBER_OF_WAYPOINTS to reflect the total number of waypoints
 * ideally the GPS will have to have 8 decimal digits or more.
 *
 * example
 * halligan front 2 point
float LATITUDES[] = { 42.40850067, 42.40808105  };
float LONGITUDES[] = { -71.11589813 , -71.11599731 };
int NUMBER_OF_WAYPOINTS = 2;
*/

/* parking lot 4 points */
// ignore one for now, 42.40824890, -71.11486816 ,
/*
float LATITUDES[] = {  42.40839004, 42.40827178, 42.40829849  };
float LONGITUDES[] = {  -71.11473083, -71.11486053, -71.11498260 };
*/

/* bromfield pearson */
float LATITUDES[] = {  42.40573120, 42.40575027, 42.40573120, 42.40579986  };
float LONGITUDES[] = {  -71.11646270, -71.11652374, -71.11646270, -71.11633300 };
int NUMBER_OF_WAYPOINTS = 4;

enum GPSSTATUS {NOSIG, NOCOURSE, GOOD};


TinyGPS gps;
Command com;

int state;
unsigned long lastCourse; // used for check validity of GPS data

struct Location {
  float latitude;
  float longitude;
  unsigned long age;
} currentLocation;

/**** For the command library ****/
int debug = 1;
String atcmd = "";

/*** pin for user control ***/
int inPin = 53;

/*** variable to indicate if we are reversing the sequence of the coordinates ***/
int reversed;

void setup()
{
  GPSsrl.begin(57600); // Baud rate of our GPS
  PCsrl.begin(57600);
  
  state = 0; //default state
  
  pinMode(inPin, INPUT);      // sets the inPin as input
  digitalWrite(inPin,HIGH);	//overwrites the default value of the pin
  reversed = 0;
}

void loop()
{
  switch (state) {
	// sanity check
	case 0:
		/* only quit while loop when proper GPS signal is acquired */
        while( ( verifyPropergps() == NOSIG ) || !checkSanity()){delay(300);}
		state = 1;
		break;
  
	// initializing connection
	case 1:
		com.start_wifi_connection();
		com.drone_is_init = com.init_drone();
		state = 2;
		break;
	
	// takeoff state
	case 2:
		com.drone_takeoff();
		com.drone_hover(1000);
		com.moveUp(1);
        com.drone_hover(500);
        // this is important because it gives you the initial bearing, don't go too slow
		com.moveForward_time(400,50); 
		state = 5;
		break;
		
	// landing
	case 3:
		doShutdown();
		state = -1;
		break;
		
	// emergency toggle
	// only used when drone is already in emergency
	case 4:
		com.drone_emergency_toggle();
        state = 2;
		break;
		
	// navigating state
	case 5:
		navigatePath();
        //delay(1000);
		state = 3;
		break;
        
		
	//stop running
	default:
		while (1) {
			//reset the default value of the pin
			digitalWrite(inPin,HIGH);
			int pinval = digitalRead(inPin);
			// if the button is pressed, the drone will start to follow the reversed sequence of the coordinates after 2 sec
			if ( pinval == 0 ) {
				reversed = !reversed;
				 com.LEDAnim(2,2);
				 delay(2000);
				 state = 2;
				  break;
			}
		}
  }
} 

/* method to read GPS data
 * return true if there is a new complete sentence from GPS
 * return false if no complete sentence exists
 */
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

/* proper way to gather GPS data and gaurantee its validity
 * return: 
 *	NOSIG if no valid recent GPS data
 * 	NOCOURSE if course is not updated from the GPS
 *	GOOD if course and coordinates are updated from GPS
 */
int verifyPropergps(){
	// the gps updates its coordinates about 5 times per second, so we want to check it less frequently
	// better to loop without delays, using millis()
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

/* main function to issue ATcommands to the drone and examine the current GPS coordinates
 * loop through the array of latitudes and longitudes
 * until the current GPS is at the last point
 * then return;
 */
void navigatePath(){
	float destinationLat; 
	float destinationLong; 
	double currentDistance;
    unsigned long gpstime,gpsdate,gpsage;
	for (int seq = 0; seq < NUMBER_OF_WAYPOINTS; seq++ ) {
		bool done = false;
		int i;
		//reverse path support
		if (reversed) { 
		  i = NUMBER_OF_WAYPOINTS - 1 - seq;
		} else {
		  i = seq;
		}
		destinationLat = LATITUDES[i];
		destinationLong = LONGITUDES[i];
		int hovercount = 0;

	while(!done)
	{        //PCsrl << "going to point " << i << " lat: " << _FLOAT(destinationLat,8) << " log: " << _FLOAT(destinationLong,8) <<endl;
		int a = verifyPropergps();
		if ( a != NOSIG) {
			currentDistance = TinyGPS::distance_between(currentLocation.latitude,currentLocation.longitude,LATITUDES[i],LONGITUDES[i]);
			//flash LEDs on drone for 1 sec to show that one destination is reached
			if(abs(currentDistance) < 3) {
				//PCsrl << "follow point " << i <<" success, current distance" << currentDistance <<endl;
				com.LEDAnim(2,1);
				com.drone_hover(1000);
				break;
			}
		}
		switch (a) {
			case NOSIG: {
				if (debug) { PCsrl << "gps data acquiring failed" <<endl;}
				com.drone_hover(200);
				hovercount++;
				// if hovercount reaches 20, means no valid GPS signal for 4 sec, quit
				if ( hovercount > 20 ) {
					return;
				}
				break;
			}
			
			case GOOD: {
				gps.get_datetime(&gpsdate,&gpstime,&gpsage);
			//PCsrl << "current gps time" << gpstime <<endl;
			//PCsrl << "current point " << i << " lat: " << _FLOAT(currentLocation.latitude,8) << " log: " << _FLOAT(currentLocation.longitude,8) <<endl;
				hovercount = 0;
				fly_to(currentLocation.latitude,currentLocation.longitude,destinationLat,destinationLong); 
				break;
			}
			
			case NOCOURSE: {
				if (debug) { PCsrl << "gps data no course update, moveforward" <<endl;}
				com.moveForward_time(500,20);
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
	/* the two movement function below dictates how long fly_to() is going to run
	 * you don't want it to be too long because that delays the checking of GPS signal
	 * and the drone might miss the target
	 */
	com.staticRotate(ceil(bearing));
	//com.moveForward(ceil(distance/3));
	/* using moveForward_time() to limit the amount of time it consumes */
    com.moveForward_time(600,40);    
	return 1;
}

/* calculate the difference between the current course of the drone's heading and the course to the destination point */
float getCourse(float startLat,float startLong,float endLat,float endLon){
  // First get the course that we need to be. 
  float courseOnCourse = TinyGPS::course_to(startLat,startLong,endLat,endLon);
  float myCourse = gps.f_course();
  return (courseOnCourse-myCourse) >= 0 ? (courseOnCourse-myCourse):(courseOnCourse-myCourse+360); 
}

/* update currentLocation */
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

/* check the distance between the start and end of the destination GPS coordinates
 * make sure it is not too far away
 */
boolean checkSanity(){
  double distanceSanity = TinyGPS::distance_between(LATITUDES[0],LONGITUDES[0],LATITUDES[NUMBER_OF_WAYPOINTS-1],LONGITUDES[NUMBER_OF_WAYPOINTS-1]);
  return distanceSanity < 1000;
}








