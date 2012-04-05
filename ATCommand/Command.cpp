#include "Arduino.h"
#include "Command.h"

extern int sequenceNumber;
extern int debug;

Command::Command()
{
  at = "";
  command = "";
  
}

String Command::sendComwdg()
{
  at = "AT*COMWDG=";
  command = at + sequenceNumber + "\r\n";
  sequenceNumber++;
  return command;
}

String Command::sendFtrim()
{
  at = "AT*FTRIM=";
  command = at + sequenceNumber + "\r\n";
  sequenceNumber++;
  return command;
}

String Command::sendConfig(String option, String value)
{
  at = "AT*CONFIG=";
  command = at + sequenceNumber + ",\"" + option + "\",\"" + value + "\"\r\n";
  sequenceNumber++;
  return command;
}

String Command::sendRef(int bit9)
{
  at = "AT*REF=";
  if(bit9 == 1){
    command = at + sequenceNumber + ",290718208\r\n"; //takeoff
  }
  else if(bit9 == 0){
    command = at + sequenceNumber + ",290717696\r\n"; //landing
  }
  // emergency -> 290717952
  sequenceNumber++;
  return command;
}

String Command::sendPcmd(int enable, int roll, int pitch, int gaz, int yaw)
{
  at = "AT*PCMD=";
  command = at + sequenceNumber + "," + enable + "," + roll + "," + pitch + "," + gaz + "," + yaw + "\r\n";
  sequenceNumber++;
  return command;
}

String Command::sendAnim(int anim, int time)
{
  at = "AT*ANIM=";
  command = at + sequenceNumber + "," + anim + "," + time + "\r\n";
  sequenceNumber++;
  return command;
}

String Command::LEDAnim(int duration)
{
  at = "AT*LED=";
  command = at + sequenceNumber + ",2,1073741824," + duration + "\r\n";
  sequenceNumber++;
  return command;
}

int Command::start_s2ip()
{
  char temp;
  delay(20000); //wait for drone to start
  
  while (ARsrl.available()){
    PCsrl.write(ARsrl.read());
  }
  
  if (debug) {
	PCsrl <<"try start s2ip"<<endl;
  }
  ARsrl.print("\r\n");
  delay(500);
  ARsrl.print("\r\n");
  delay(500);
  ARsrl << "cd ~"<<endl;
  if (debug) {
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
      PCsrl << "s2ip is running" <<endl;
      ARsrl << "bullshit\r\n"; //to fix a delay bug
      break;
    }
    //PCsrl<<"s2ip not running" <<endl;
  }
  return 1;
}

void Command::quit_s2ip()
{
  ARsrl.println("EXIT");
  while (ARsrl.available () ) {
    PCsrl.write( ARsrl.read() ); 
  }
}

int Command::init_drone() {
	ARsrl << sendComwdg();
	ARsrl << sendFtrim();
	ARsrl << sendConfig("general:navdata_demo","TRUE");
	ARsrl << sendConfig("control:altitude_max","2000");
	ARsrl << sendRef(1);
	
		//do some checking??
}

int Command::drone_hover() {
	while (!drone_is_hover) {
		ARsrl<< sendPcmd(1,0,0,0,0);
			// do some checking
	}

}
