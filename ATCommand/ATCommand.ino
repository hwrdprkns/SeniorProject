#include "Command.h"
#include "Streaming.h"

int debug = 1;
extern ring_buffer rx_buf;
extern resultint_ resultint;

Command com;
String atcmd = "";

#include "TimerThree.h"
#define LEDpin 13

void setup()
{
  PCsrl.begin(9600);
  if (debug) {
    // never use three ! together in arduino code
    PCsrl << "Whatever!\r\n";
  }
  
  com.start_wifi_connection();

  com.drone_is_init = com.init_drone();
}



void loop()
{  
  if (com.drone_is_init == 0) {
        if (debug) {
      // never use three ! together in arduino code
      PCsrl << "Drone wasn't initlized before loop() was called. Initalizing now.\r\n";
    }
  } else {
    //com.LEDAnim(2,3);
    
	com.drone_takeoff();
	//com.sendRef(LANDING);
    //com.sendRef(TAKEOFF);
    
	//com.drone_hover(5000);
    /*
    for (int i=0; i<100; i++) {
      com.moveRotate(360);
      delay(50);
    }*/
    
	com.drone_hover(2000);
	com.moveLeft(1);
	com.drone_hover(2000);
	com.LEDAnim(2,3);
	com.moveRight(1);
	com.drone_hover(2000);
	//com.moveDown(1);
	/*com.moveBackward(1);
	com.drone_hover(2000);*/
    /*delay(50);
    
    delay(500);
    */

	com.drone_landing();

    delay(500);
    
    // end of program
    while (1) {};
  }
}
