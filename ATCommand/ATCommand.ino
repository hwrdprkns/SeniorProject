#include "Command.h"
#include "Streaming.h"

int debug = 1;
extern ring_buffer rx_buf;
extern resultint_ resultint;

Command com;
int sequenceNumber = 1;
String atcmd = "";

#include "TimerOne.h"
#define LEDpin 13



void setup()
{
  PCsrl.begin(9600);

  com.start_wifi_connection();
  
  com.drone_is_init = com.init_drone();
  
  Timer1.initialize(COMWDG_INTERVAL_USEC);
  Timer1.attachInterrupt(watchdog_timer);
  
}

void watchdog_timer() {
  com.sendwifi(com.makeComwdg());
}

void loop()
{  
	  
  if (com.drone_is_init == 0) {
        if (debug) {
      // never use three ! together in arduino code
      PCsrl.println("Drone wasn't initlized before loop() was called. Initalizing now.\r\n");
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
    
    com.drone_takeoff();
    com.drone_takeoff();
    
    com.sendwifi(com.makePcmd(1,0,0,0,0));
    com.sendwifi(com.makePcmd(1,0,0,0,0));
    delay(5000);
    
    com.moveForward(1);
    com.moveRotate(180);
    com.moveForward(1);
    com.moveRotate(180);
    
    delay(500);
    
    com.drone_landing();
    com.drone_landing();
    
    delay(500);
    
    //end of program
    Timer1.detachInterrupt();
    PCsrl.println("Program finished");
    while (1){};
    
  }
}

