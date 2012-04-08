#include "Command.h"
#include "Streaming.h"

int debug = 1;

Command com;
int sequenceNumber = 1;
int i = 1;
String atcmd = "";

#include "TimerThree.h"
#define LEDpin 13

void setup()
{
  ARsrl.begin(115200);
  PCsrl.begin(115200);
  if (debug) {
	//never use three ! together in arduino code
	PCsrl << "yeahhh!!fuckya!\r\n";
  }
  Timer3.initialize(SERIAL_INTERVAL_USEC);
  Timer3.attachInterrupt(SrlRead);
}



void loop()
{
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
