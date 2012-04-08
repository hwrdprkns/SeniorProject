#include "Command.h"
#include "Streaming.h"

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
  ARsrl.begin(115200);
  PCsrl.begin(115200);
  if (debug) {
    //never use three ! together in arduino code
    PCsrl << "Whatever!\r\n";
  }
  Timer3.initialize(SERIAL_INTERVAL_USEC);
  Timer3.attachInterrupt(SrlRead);
}

void loop()
{
  if (com.s2ip_running == 0) {
    read_rx_buf();
   com.s2ip_running = com.start_s2ip();
   //upon exit s2ip_running == 1
  }
  
  if (com.s2ip_running == 1) {
    delay(200);
    
    if (com.drone_is_init == 0) {
      com.drone_is_init = com.init_drone();
      read_rx_buf();
    }
    
    // drone take off
    if (com.drone_is_init == 1) {
      if (debug) {
        PCsrl << "drone is init" <<endl;
      }
      com.drone_takeoff();
      read_rx_buf();
     //com.drone_hover();
    delay(1000);
    
    int i = 0;
    for ( i = 0; i<10; i++ ) {
      com.drone_hover();
      delay(100);
    }
    //com.drone_move();
    delay(1000);
 com.drone_landing();
    delay(500);  
  }
    delay(500);
    
    com.quit_s2ip();
    com.s2ip_running == 0;
    delay(100000);
  }
}
