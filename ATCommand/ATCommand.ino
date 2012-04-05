#include "Command.h"
#include "Streaming.h"

// how can I keep all flag variable in one place? in Command class private variable or in ATcommand.ino?
int debug = 1;
int s2ip_running = 0;
//int drone_is_hover;
int drone_is_init;
Command com;
int sequenceNumber = 1;
int i = 1;
String atcmd = "";


void setup()
{
  ARsrl.begin(115200);
  PCsrl.begin(115200);
  if (debug) {
	//never use three ! together in arduino code
	PCsrl << "yeahhh!!fuckya!\r\n";
  }
  
     drone_is_init = 0;
     drone_is_hover = 0;
}

void loop()
{
  if ( s2ip_running == 0 ) {
   s2ip_running = com.start_s2ip();
   //upon exit s2ip_running == 1
  }
  
  if ( s2ip_running == 1) {
    delay(200);
    ARsrl << "AT*LED=1,2,1073741824,3\r\n";
	delay(3000); 
    com.quit_s2ip();
    s2ip_running == 0;
    delay(100000);
	
	/* code that run for take off ?*/
	/*
	if (drone_is_init == 0) {
		drone_is_init = com.init_drone();
	}
	// drone take off
	if (drone_is_init == 1) {
		com.drone_hover();
	}
	*/
  }
  
  
  
}
