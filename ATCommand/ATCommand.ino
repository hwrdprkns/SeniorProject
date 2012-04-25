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
  com.sendwifi(com.sendComwdg(0));
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
    
    /* reached 
    read_rx_buf();
    delay(40);
    delay(1000000);
    */
  } else {
    com.LEDAnim(2,3);
    
    com.sendRef(TAKEOFF);
    com.sendRef(TAKEOFF);
    
    com.sendwifi(com.makePcmd(1,0,0,0,0));
    com.sendwifi(com.makePcmd(1,0,0,0,0));
    delay(10000);
    
<<<<<<< HEAD
    for (int i =0;i<100;i++) {
      com.moveRotate(360);
      delay(50);
    }
=======
    
    com.moveForward(1);
    delay(50);
>>>>>>> 3a670c48e0eb66684a54b68ac9812ec0ca500168
    delay(500);
    
    com.sendRef(LANDING);
    com.sendRef(LANDING);
    
    delay(500);
    
    // end of program
    Timer3.detachInterrupt();
    while (1) {};
  }
}
