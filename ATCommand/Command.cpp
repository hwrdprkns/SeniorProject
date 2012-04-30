#ifndef GAINSPAN
#define GAINSPAN
#include "Arduino.h"
#include "Command.h"

int sequenceNumber = 1;
extern int debug;

ring_buffer rx_buf= {{0}, 0, 0};
resultint_ resultint;

Command::Command()
{
  at = "";
  command = "";
  s2ip_running = 0;
  drone_is_init = 0;
}

/************************************
* low level routine
************************************/
void Command::sendwifi(String s)
{
  WIFIsrl.write(27); //esc
  WIFIsrl.print("S0"); //choose connection CID 0
  WIFIsrl.print(s);
  WIFIsrl.write(27);
  WIFIsrl.print("E");
  
  if (debug) PCsrl << s;
}

void Command::sendComwdg(int msec)
{
  at = "AT*COMWDG=";
  
  for (int i=0; i<msec; i+=30) {
    command = at + sequenceNumber + "\r\n";
	sequenceNumber++;
    sendwifi(command);
    delay(30);
  }
}

void Command::sendFtrim()
{
  at = "AT*FTRIM=";
  command = at + sequenceNumber + "\r\n";
  sequenceNumber++;
  #ifndef GAINSPAN
    ARsrl << command;
  #else
    sendwifi(command);
  #endif
}

void Command::sendConfig(String option, String value)
{
  at = "AT*CONFIG=";
  command = at + sequenceNumber + ",\"" + option + "\",\"" + value + "\"\r\n";
  sequenceNumber++;
  #ifndef GAINSPAN
    ARsrl << command;
  #else
    sendwifi(command);
  #endif
}

void Command::sendRef(flying_status fs)
{
  at = "AT*REF=";
  if (fs == TAKEOFF) {
    command = at + sequenceNumber + ",290718208\r\n"; //takeoff
  } else if (fs == LANDING) {
    command = at + sequenceNumber + ",290717696\r\n"; //landing
  } else if (fs == EMERGENCY_TOGGLE) {
    command = at + sequenceNumber + ",290717952\r\n"; //emergency toggle
  }
  sequenceNumber++;
  if (debug) {
    PCsrl << command << endl;
  }
    sendwifi(command);
}

void Command::send_control_commands()
{
  at = "AT*CTRL=";
  sendwifi(at+sequenceNumber+",4,0\r\n");
  sequenceNumber++;
  sendwifi(at+sequenceNumber+",0,0\r\n");
  sequenceNumber++;
  sendwifi(at+sequenceNumber+",4,0\r\n");
  sequenceNumber++;
}

String Command::makePcmd(int enable, float roll, float pitch, float gaz, float yaw)
{
  at = "AT*PCMD=";
  command = at + sequenceNumber + "," + enable + "," + fl2int(roll) + "," + fl2int(pitch) + "," + fl2int(gaz) + "," + fl2int(yaw) + "\r";
  sequenceNumber++;
    sendwifi(command);
  return command;
}

String Command::makeAnim(anim_mayday_t anim, int time)
{
  at = "AT*ANIM=";
  command = at + sequenceNumber + "," + anim + "," + time + "\r\n";
  sequenceNumber++;
  return command;
}

void Command::LEDAnim(int animseq, int duration)
{
  at = "AT*LED=";
  command = at + sequenceNumber + "," + animseq + ",1073741824," + duration + "\r\n";
  sequenceNumber++;
    sendwifi(command);

}


/************************************
* high level routine
************************************/
int Command::start_wifi_connection()
{
  WIFIsrl.begin(9600);
  
  WIFIsrl.println("");
  WIFIsrl.println("AT&F");
  WIFIsrl.println("ATE1"); //turn on echo
  WIFIsrl.println("AT+NCLOSEALL");
  WIFIsrl.print("AT+NMAC=00:1d:c9:10:39:6f\r"); //set MAC address
  WIFIsrl.println("AT+WM=0");
  WIFIsrl.println("AT+NDHCP=1");
  
   char c1, c2;
  // drone's network profile, change if needed
  delay(1000);
  readARsrl();
  bool done = false;
  while ( !done) {
	while( WIFIsrl.available() ) {
		c1 = WIFIsrl.read();
		if ( c1 == 'I' ) {
			c2 = WIFIsrl. read();
			if ( c2 == 'P' )  {done = true; break;}
		}
	}
  	WIFIsrl.println("AT+WA=ardrone_279440");
    //WIFIsrl.println("AT+WA=ardrone_154516");
	delay(1500);
	}
  WIFIsrl.println("AT+NCUDP=192.168.1.1,5556");
  readARsrl();
  delay(1000); //need 1 seconds for connection to establish
  WIFIsrl.println("ATE0"); //turn off echo
  return 0;
}


//NOTES: this toggles the emergency flag. 
void Command::drone_emergency_toggle()
{
	String resetcmd;
  at = "AT*REF=";
  command = at + sequenceNumber + ",290717696\r";
  resetcmd = command;
  sequenceNumber++;
  at = "AT*REF=";
  command = at + sequenceNumber + ",290717952\r";
  resetcmd = resetcmd+command;
  sequenceNumber++;
  #ifndef GAINSPAN
    ARsrl << resetcmd;
  #else
    sendwifi(resetcmd);
  #endif
}

// Movement functions
int Command::moveForward(float distanceInMeters)
{
  float i = 0;
  String moveForward = makePcmd(1, 0, -.5, 0, 0);
  #ifndef GAINSPAN
    ARsrl << moveForward;
  #else
    sendwifi(moveForward);
  #endif
  for ( i = 0; i < distanceInMeters; ) {
  sendComwdg(60);
  i = i+0.2;
  }
  moveForward = makePcmd(1, 0, 0, 0, 0);
  sendwifi(moveForward);
  return 1;
}

int Command::moveBackward(float distanceInMeters)
{
  float i = 0;
  String moveForward = makePcmd(1, 0, .5, 0, 0);
  #ifndef GAINSPAN
    ARsrl << moveForward;
  #else
    sendwifi(moveForward);
  #endif
  for ( i = 0; i < distanceInMeters; ) {
  sendComwdg(60);
  i = i+0.15;
  }
  moveForward = makePcmd(1, 0, 0, 0, 0);
  sendwifi(moveForward);
  return 1;
}

int Command::moveUp(float distanceInMeters)
{
  float i = 0;
  String move;
  for ( i = 0; i < distanceInMeters; ) {
  move = makePcmd(1, 0, 0, 0.9, 0);
  sendwifi(move);
  sendComwdg(30);
  delay(80);
  i = i+0.1;
  }
  return 1;
}

int Command::moveDown(float distanceInMeters)
{
  float i = 0;
  String move;
  for ( i = 0; i < distanceInMeters; ) {
  move = makePcmd(1, 0, 0, -0.8, 0);
  sendwifi(move);
  delay(70);
  sendComwdg(30);
  i = i+0.1;
  }
  move = makePcmd(1,0,0,0,0);
  sendwifi(move);
  delay(30);
  return 1;
}

int Command::moveLeft(float distanceInMeters)
{
  float i = 0;
  String move;
  move = makePcmd(1, -0.6, 0, 0, 0);
  sendwifi(move);
  for ( i = 0; i < distanceInMeters; ) {
  sendComwdg(30);
  i = i+0.15;
  }
  move = makePcmd(1,0,0,0,0);
  sendwifi(move);
  delay(30);
  return 1;
}

int Command::moveRight(float distanceInMeters)
{
  float i = 0;
  String move;
  move = makePcmd(1, 0.6, 0, 0, 0);
  sendwifi(move);
  for ( i = 0; i < distanceInMeters; ) {
  sendComwdg(30);
  i = i+0.15;
  }
  move = makePcmd(1,0,0,0,0);
  sendwifi(move);
  delay(30);
  return 1;
}

int Command::moveRotate(int yawInDegrees)
{
  int i = 0;
  int sign;
  if ( yawInDegrees >= 0 ) sign = 1;
  else sign = -1;
    if ( yawInDegrees >= 360 ) yawInDegrees -= 360;
	if ( yawInDegrees <= -360) yawInDegrees += 360;
  //(sign*yawInDegrees) is always positive
  while (i < (sign*yawInDegrees) ) {
    String moveRotate = makePcmd(1, 0, 0, 0, (0.3*sign));
    sendwifi(moveRotate);
    delay(150);
    i += 16;
  }
  return 1;
}

void Command::drone_takeoff() {
	for (int i = 0; i < 100; i++ ){
		sendRef(TAKEOFF);
		delay(30);
	}
}
	
void Command::drone_landing() {
	for (int i = 0; i < 60; i++ ){
		sendRef(LANDING);
    	delay(30);
	}
}

int Command::start_s2ip()
{
  char temp;
  //delay(20000); //wait for drone to start
  
  readARsrl();
  
  if (debug) {
    PCsrl << "trying to start s2ip" << endl;
  }
  
  ARsrl.print("\r\n");
  delay(500);
  ARsrl.print("\r\n");
  delay(500);
  ARsrl << "cd ~" << endl;
  
  if (debug) {
    readARsrl();
  }
  
  delay(500);
  ARsrl << "cd data/video/apps/" << endl;
  delay(500);
  ARsrl << "./s2ip.arm" << endl; 
  while ((int) temp != 2) {
    temp = ARsrl.read();
    if (temp == 2) {
      PCsrl << "s2ip is running" << endl;
      ARsrl << "bullshit\r\n"; //to fix a delay bug
      break;
    }
    //PCsrl << "s2ip not running" << endl;
  }
  if (debug) {
    while (ARsrl.available()) {
      PCsrl.write(ARsrl.read());
    }
  }
  return 1;
}

void Command::quit_s2ip()
{
  ARsrl.println("EXIT");
  while (ARsrl.available()) {
    PCsrl.write(ARsrl.read()); 
  }
}

int Command::init_drone()
{
  PCsrl << "I'm initing\r\n";
  sendConfig("general:navdata_demo","TRUE");
  sendConfig("control:altitude_max","3000");
  sendConfig("control:euler_angle_max","0.35"); //between 0 and 0.52
  sendConfig("control:outdoor","FALSE"); // keep this false to maintain the flight param consistant
  sendConfig("control:flight_without_shell","FALSE");
  send_control_commands();
  //sendComwdg(100);
  //drone_emergency_toggle();
  sendFtrim();
  delay(50);

  return 1;
}

void Command::drone_hover(int msec)
{
  sendwifi(makePcmd(0, 0, 0, 0, 0));
  sendComwdg(msec);
}

long Command::fl2int(float value)
{
  resultint.i = 0;
  if (value < -1 || value > 1) {
    resultint.f = 1;
  } else {
    resultint.f=value;
  }

  return resultint.i;
}

void Command::readARsrl()
{
  while (ARsrl.available()) {
    char c = ARsrl.read();
    if (debug) {
      PCsrl.write(c);
    }
  }
}


/*
void SrlRead()
{
  if (inService) {
    PCsrl.println("timer kicked too fast");
    return;
  }
  interrupts();
  inService = true;
  while(ARsrl.available()) {
    unsigned char k = ARsrl.read();
    store_char(k, &rx_buf);
  }
  inService = false;
}*/

void read_rx_buf()
{
  while (rx_buf.tail != rx_buf.head) {
    if (debug) {
      PCsrl.write(rx_buf.buffer[rx_buf.tail]);
    }
    rx_buf.tail = (unsigned int) (rx_buf.tail+ 1) % SERIAL_BUFFER_SIZE;
  }
}

inline void store_char(unsigned char c, ring_buffer *buffer)
{
  int i = (unsigned int)(buffer->head + 1) % SERIAL_BUFFER_SIZE;

  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if (i != buffer->tail) {
    buffer->buffer[buffer->head] = c;
    buffer->head = i;
  }
  else {
    Serial.println("ring buffer is too small");
  }
}

#endif
