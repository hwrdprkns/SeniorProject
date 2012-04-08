#include "WayPoint.h"
#include "TinyGps.h"
#include <math.h>
#include <SoftwareSerial.h>
#include "Command.h"
#include "Streaming.h"

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



/** Command definitions **/

int debug = 1;
extern ring_buffer rx_buf;
Command com;
int sequenceNumber = 1;
int i = 1;
String atcmd = "";
#include "TimerThree.h"
#define LEDpin 13



void setup()
{
  Serial2.begin(57600); // Baud rate of our GPS
  commandSetup();
}
 


void loop()
{
  checkSanity();
  commandLoop();
  navigatePath(0,WayPoint::calculateDistance(getLatitudeFromGPS(),getLongitudeFromGPS(),LATITUDES[NUMBER_OF_WAYPOINTS-1],LONGITUDES[NUMBER_OF_WAYPOINTS-1]);
  return;
} 


/*** Setting up command library here **/

void commandSetup(){
  ARsrl.begin(115200);
  PCsrl.begin(115200);
  if (debug) {
	//never use three ! together in arduino code
	PCsrl << "yeahhh!!fuckya!\r\n";
  }
  Timer3.initialize(SERIAL_INTERVAL_USEC);
  Timer3.attachInterrupt(SrlRead);
}


void commandLoop(){
    if ( com.s2ip_running == 0 ) {
   com.s2ip_running = com.start_s2ip();
   //upon exit s2ip_running == 1
  }
  
  if ( com.s2ip_running == 1) {
    delay(200);
    ARsrl<< com.LEDAnim(5);
	delay(3000); 
	
	/* code that run for take off ?*/
	if (com.drone_is_init == 0) {
		com.drone_is_init = com.init_drone();
	}
	// drone take off
	if (com.drone_is_init == 1) {
		com.drone_takeoff();
	}
	delay(500);
	com.drone_landing();

	com.quit_s2ip();
    com.s2ip_running == 0;
    delay(100000);
  }
}




// ** End command setup **//



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

  //boolean droneSanity = checkDroneSanity();

  boolean isSaneDistance = distanceSanity < 1000;

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






