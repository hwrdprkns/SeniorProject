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
  //Timer3.initialize(SERIAL_INTERVAL_USEC);
  //Timer3.attachInterrupt(SrlRead);
  
  com.start_wifi_connection();
  com.drone_is_init = com.init_drone();
  com.drone_takeoff();
  com.drone_landing();
  
}

void loop()
{   
  
  
    if (com.drone_is_init == 0) {
      
      
      PCsrl << "I'm in the positive condition\r\n";
      
      //ARsrl << com.LEDAnim(2,3);
      //ARsrl << "AT*CONFIG=1,\"control:altitude_max\",\"2000\"";
      
      com.init_drone();
      
      read_rx_buf();
      delay(1000);
    }else {
      
      PCsrl << "I'm in the negative condition\r\n";
      
      com.drone_takeoff();
      
      read_rx_buf();

      delay(5000);

      com.drone_landing();
      delay(500);
    
    
    com.s2ip_running == 0;

  }
}

