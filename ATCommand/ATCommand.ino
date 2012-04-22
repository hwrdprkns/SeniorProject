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

/*void watchdog_timer() {
  ARsrl << com.makeComwdg();
}*/

void setup()
{
  PCsrl.begin(9600);
  if (debug) {
    // never use three ! together in arduino code
    PCsrl << "Whatever!\r\n";
  }
  
  com.start_wifi_connection();

  
  //ARsrl << com.sendRef(LANDING);
  //Timer3.initialize(COMWDG_INTERVAL_USEC);
  //Timer3.attachInterrupt(watchdog_timer);
}

void loop()
{  
	com.sendComwdg_t(100);
	
  delay(100000); //turn off
    
  if (com.drone_is_init == 0) {
        if (debug) {
      // never use three ! together in arduino code
      PCsrl << "initializing Drone\r\n";
    }
      com.drone_is_init = com.init_drone();
      delay(30);
      com.drone_takeoff();
      delay(100);
      // delay(3000);
      com.drone_landing();
      delay(30);
      ARsrl << com.makeComwdg();
      delay(30);
      ARsrl << com.LEDAnim(2,3);
  
    //PCsrl << "Drone initialization failed, please check connection\r\n";
    
    /*  reached */
    read_rx_buf();
    delay(40);
    //delay(1000000);
    
  } else {
    if (debug) {
      // never use three ! together in arduino code
      PCsrl << "Drone initialized\r\n";
    }
    
    ARsrl << com.LEDAnim(2,3);
    
    com.drone_takeoff();
    //com.drone_move_up(100);
    
    read_rx_buf();
    
    /*com.sendComwdg_t(400);
    com.moveForward(1);
    com.sendComwdg_t(400);*/
    
    //com.drone_hover(2000);
    
    //delay(5000);
    
    com.drone_landing();
    delay(500);
    
    //end of program
    delay(400000);
  }
  
}

