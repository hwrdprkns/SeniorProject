#include "Command.h"
#include "Streaming.h"

int debug = 1;
extern ring_buffer rx_buf;
extern resultint_ resultint;

Command com;
int sequenceNumber = 1;
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

  Timer3.initialize(COMWDG_INTERVAL_USEC);
  com.drone_is_init = com.init_drone();
  
  Timer3.attachInterrupt(watchdog_timer);
  
}

void watchdog_timer() {
  com.sendwifi(com.makeComwdg());
}

void loop()
{  
	  
  if (com.drone_is_init == 0) {
        if (debug) {
      // never use three ! together in arduino code
      PCsrl << "Drone wasn't initlized before loop() was called. Initalizing now.\r\n";
    }
    

    
    /*
      delay(30);
      com.drone_takeoff();
      delay(100);
      // delay(3000);
      com.drone_landing();
      delay(30);
      ARsrl << com.makeComwdg();
      delay(30);
      ARsrl << com.LEDAnim(2,3);
      
      */
  
    //PCsrl << "Drone initialization failed, please check connection\r\n";
    
    /*  reached 
    read_rx_buf();
    delay(40);
    //delay(1000000);
    */
    
  } else {
     
    com.doLEDAnim(2,3);
    
    com.drone_takeoff();
    com.drone_takeoff();
    
    com.sendwifi(com.makePcmd(1,0,0,0,0));
    com.sendwifi(com.makePcmd(1,0,0,0,0));
    delay(10000);
    
    
    com.moveForward(1);
    delay(50);
    delay(500);
    
    com.drone_landing();
    com.drone_landing();
    
    delay(500);
    
    //end of program
    Timer3.detachInterrupt();
    while (1){};
    
  }
}

