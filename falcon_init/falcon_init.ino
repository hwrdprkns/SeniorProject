
#include "Streaming.h"

#define ARsrl Serial1
#define PCsrl Serial

int sequenceNumber = 1;
int i = 1;
String atcmd = "";

int s2ip_running = 0;


void start_s2ip()
{
  char temp;
  delay(20000); //wait for drone to start
  
  while (ARsrl.available()){
    PCsrl.write(ARsrl.read());
  }
  
  PCsrl<<"try start s2ip"<<endl;
  ARsrl.print("\r\n");
  delay(500);
  ARsrl.print("\r\n");
   delay(500);
  ARsrl << "cd ~"<<endl;
  if (verbose) {
  while (ARsrl.available()){
    PCsrl.write(ARsrl.read());
  }
  }
  delay(500);
  ARsrl << "cd data/video/apps/"<<endl;
  delay(500);
  ARsrl <<"./s2ip.arm -v"<<endl; 
  while ( (int) temp != 2) {
    temp = ARsrl.read();
    if ( temp == 2 ) {
      s2ip_running = 1;
      PCsrl << "s2ip is running" <<endl;
      ARsrl << "bullshit\r\n"; //to fix a delay bug
      break;
    }
    //PCsrl<<"s2ip not running" <<endl;
  }
}

void quit_s2ip()
{
  ARsrl.println("EXIT");
  while (ARsrl.available () ) {
    PCsrl.write( ARsrl.read() ); 
  }
}



void setup()
{
  ARsrl.begin(115200);
  PCsrl.begin(115200);
  
  PCsrl << "yeahhh!!fuckya!\r\n";
}

void loop()
{

  if ( s2ip_running == 0 ) {
   start_s2ip();
  }
  
  if ( s2ip_running == 1) {
    delay(200);
    ARsrl << "AT*LED=1,2,1073741824,3\r\n";
   delay(3000); 
    quit_s2ip();
    s2ip_running == 0;
    delay(100000);
  }

}
