#include "Arduino.h"
#include "Command.h"

extern int sequenceNumber;
extern int debug;

Command::Command()
{
  at = "";
  command = "";
     s2ip_running = 0;
    drone_is_init = 0;
    drone_is_hover = 0;
	emergency = 0;
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
  command = at + sequenceNumber + ",\r\n";
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

String Command::sendRef(flying_status fs)
{
  at = "AT*REF=";
  if(fs == TAKEOFF){
    command = at + sequenceNumber + ",290718208\r\n"; //takeoff
  }
  else if(fs == LANDING){
    command = at + sequenceNumber + ",290717696\r\n"; //landing
  }
  // emergency -> 290717952
  sequenceNumber++;
  return command;
}

String Command::sendRef(flying_status fs, int emergency) {
	if (emergency == 1) {
		String emergency_ready = sendRef(LANDING);
		command = at + sequenceNumber + ",290717952\r\n";
		sequenceNumber++;
		return emergency_ready + command;
	}
}

// public with float version
String Command::makePcmd(int enable, float roll, float pitch, float gaz, float yaw) {
	at = "AT*PCMD=";
  	command = at + sequenceNumber + "," + enable + "," + fl2int(roll) + "," + fl2int(pitch) + "," + fl2int(gaz) + "," + fl2int(yaw) + "\r\n";
  	sequenceNumber++;
	return command;
}

/** Movement functions **/

int Command::moveStraightForward(int distanceInMeters){
	int pitchCalibrationFactor = -1;
	int delayFactor = 3;
	String moveForward = makePcmd(1,lastRoll,(lastPitch - pitchCalibrationFactor),lastGaz,lastYaw);
	sendPcmd(command);
	delay(delayFactor*distanceInMeters*50); //This is in millis, right?
	String hover = makePcmd(1,lastRoll,lastPitch,lastGaz,lastYaw);
	sendPcmd(hover);
	return 1;
}

int Command::moveRotate(float yawInDegrees){
	int rotateCalibrationFactor = 5;
	int delayFactor = 3;
	String moveRotate = makePcmd(1,lastRoll,lastPitch,lastGaz,(lastYaw + yawInDegrees*rotateCalibrationFactor));
	sendPcmd(moveRotate);
	delay(delayFactor*rotateCalibrationFactor*50);//Again... in millis?
	String hover = makePcmd(1,lastRoll,lastPitch,lastGaz,lastYaw);
	sendPcmd(hover);
	return 1;
}

void Command::sendPcmd(String command){
	previousCommand = command;
	ARsrl << command;
}


String Command::makeAnim(int anim, int time)
{
  at = "AT*ANIM=";
  command = at + sequenceNumber + "," + anim + "," + time + "\r\n";
  sequenceNumber++;
  return command;
}

String Command::LEDAnim(int duration)
{
	PCsrl << "calling LEDAnime" <<endl;
  at = "AT*LED=";
  command = at + sequenceNumber + ",2,1073741824," + duration + "\r\n";
  sequenceNumber++;
  return command;
}

int Command::start_s2ip()
{
  char temp;
  delay(20000); //wait for drone to start
  
	readARsrl();
  
  if (debug) {
	PCsrl <<"try start s2ip"<<endl;
  }
  ARsrl.print("\r\n");
  delay(500);
  ARsrl.print("\r\n");
  delay(500);
  ARsrl << "cd ~"<<endl;
  if (debug) {
	readARsrl();
  }
  delay(500);
  ARsrl << "cd data/video/apps/"<<endl;
  delay(500);
  ARsrl <<"./s2ip.arm"<<endl; 
  while ( (int) temp != 2) {
    temp = ARsrl.read();
    if ( temp == 2 ) {
      PCsrl << "s2ip is running" <<endl;
      ARsrl << "bullshit\r\n"; //to fix a delay bug
      break;
    }
    //PCsrl<<"s2ip not running" <<endl;
  }
  if (debug) {
	while (ARsrl.available()){
		PCsrl.write(ARsrl.read());
	}
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
	ARsrl << sendRef(LANDING,1); //clear emergency flag
	emergency = 0;
	
		//do some checking??
}

int Command::drone_takeoff() {
	ARsrl << sendRef(TAKEOFF);
	int i = 0;
	while (!drone_is_hover && !emergency) {
		ARsrl<< makePcmd(1,(float) 0, (float) 0, (float) 1, (float) 0);
		delay(30);
			// do some checking
		i++;
		if (i == 5) drone_is_hover=1;
	}
}

int Command::drone_hover() {
	while (!emergency) {
		ARsrl<< makePcmd(1,(float) 0, (float) 0, (float) 0, (float) 0);
		delay(30);
			// do some checking
	}
}

int Command::drone_landing() {
	float neg_one = -1.0;
	int i= 0;
	while (drone_is_hover && !emergency) {
		ARsrl<< makePcmd(1,(float)0,(float)0,neg_one,(float)0);
		delay(30);
			// do some checking
		i++;
		if (i == 5) drone_is_hover=0;
	}
}

int Command::fl2int(float value) {
	int resultint = 0;
	if ( value < -1 || value > 1 ) {
		resultint = 1;
	} else {
		resultint = *(int*)(&value);
	}
	return resultint;
}

void Command::readARsrl() {
	while (ARsrl.available()){
		if (debug) {
			PCsrl.write(ARsrl.read());
		}
	}
}
