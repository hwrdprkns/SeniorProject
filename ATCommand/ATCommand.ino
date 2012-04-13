#include "Command.h"
#include "Streaming.h"

int debug = 1;
extern ring_buffer rx_buf;
extern resultint_ resultint;

Command com;
int sequenceNumber = 1;
int i = 1;
String atcmd = "";

#include "TimerThree.h"
#define LEDpin 13

void setup()
{
  PCsrl.begin(115200);
  if (debug) {
    // never use three ! together in arduino code
    PCsrl << "Whatever!\r\n";
  }
  Timer3.initialize(SERIAL_INTERVAL_USEC);
  Timer3.attachInterrupt(SrlRead);
  
  com.start_wifi_connection();
}

void loop()
{
    if (com.drone_is_init == 0) {
      ARsrl << com.LEDAnim(2,3);
      com.drone_is_init = com.init_drone();
      
      read_rx_buf();
      delay(1000);
    }
    
    // drone take off
    if (com.drone_is_init == 1) {
      if (debug) {
        PCsrl << "drone is init" << endl;
      }
      com.drone_takeoff();
      read_rx_buf();
      
      //com.moveForward(4);
      ARsrl << com.LEDAnim(1,1);
      //com.moveRotate(90);
      delay(1000);
      com.drone_hover(3000);

      //com.moveForward(4);
      /*int i = 0;
      for (i=0; i<10; i++) {
        com.drone_hover();
        delay(100);
      }*/
      //com.drone_move();
      //delay(1000);
      com.drone_landing();
      delay(500);
    
    
    com.s2ip_running == 0;
    delay(200000);
  }
}

