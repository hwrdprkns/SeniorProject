#ifndef GAINSPAN
#define GAINSPAN
#include "Arduino.h"
#include "Command.h"

extern int sequenceNumber;
extern int debug;

ring_buffer rx_buf= {{0}, 0, 0};
resultint_ resultint;

Command::Command()
{
  at = "";
  command = "";
  s2ip_running = 0;
  drone_is_init = 0;
  drone_is_hover = 0;
  emergency = 0;
}

void Command::sendwifi(String s)
{
  WIFIsrl.write(27); //esc
  WIFIsrl.print("S0"); //choose connection CID 0
  WIFIsrl.print(s);
  WIFIsrl.write(27);
  WIFIsrl.print("E");
  
  if (debug) PCsrl << s;
}

int Command::start_wifi_connection()
{
  WIFIsrl.begin(9600);
  
  // abandoned along with autoconnection mode
  //WIFIsrl.print("+++");
  //delay(1250);
  
  #ifndef GAINSPAN
    PCsrl << "no gainspan defined" << endl;
  #else
    PCsrl << "gainspan defined" << endl;
  #endif
  
  WIFIsrl.println("");
  WIFIsrl.println("AT&F");
  //WIFIsrl.println("ATE0"); //turn off echo
  WIFIsrl.print("AT+NMAC=00:1d:c9:10:39:6f\r"); //set MAC address
  WIFIsrl.println("AT+WM=0");
  WIFIsrl.println("AT+NDHCP=1");
  
  // drone's network profile, change if needed
  WIFIsrl.println("AT+WA=ardrone_279440");
  WIFIsrl.println("AT+NCUDP=192.168.1.1,5556");
  readARsrl();
  
  delay(3000); //need 3 seconds for connection to establish
  return 0;
}

String Command::sendComwdg(int msec)
{
  at = "AT*COMWDG=";
  command = at + sequenceNumber + "\r\n";
  sequenceNumber++;
  
  for (int i = 0; i < msec; i += 20) {
    #ifndef GAINSPAN
      ARsrl << command;
    #else
      sendwifi(command);
    #endif
    delay(20);
  }
  return command;
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
  #ifndef GAINSPAN
    ARsrl << command;
  #else
    sendwifi(command);
  #endif
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

void Command::drone_emergency_reset()
{
  at = "AT*REF=";
  command = at + sequenceNumber + ",290717952\r\n";
  sequenceNumber++;
  #ifndef GAINSPAN
    ARsrl << command;
  #else
    sendwifi(command);
  #endif
}

// Movement functions
int Command::moveForward(float distanceInMeters)
{
  float i = 0;
  String moveForward = makePcmd(1, 0, -.855, 0, 0);
<<<<<<< HEAD
  #ifndef GAINSPAN
    ARsrl << moveForward;
  #else
    sendwifi(moveForward);
  #endif
  /*while (i < distanceInMeters) {
    String stayForward = makePcmd(1, 0, -.500, 0, 0);
    sendPcmd(stayForward);
    delay(200);
    i += 0.2;
  }*/
=======
  sendPcmd(moveForward);
>>>>>>> 3a670c48e0eb66684a54b68ac9812ec0ca500168
  return 1;
}

int Command::moveRotate(float yawInDegrees)
{
  int i = 0;
  while (i < yawInDegrees) {
    String stayRotate = makePcmd(1, 0, 0, 0, 0.17);
    #ifndef GAINSPAN
      ARsrl << stayRotate;
    #else
      sendwifi(stayRotate);
    #endif
    delay(150);
    i += 8;
  }
  return 1;
}

String Command::makePcmd(int enable, float roll, float pitch, float gaz, float yaw)
{
  at = "AT*PCMD=";
  command = at + sequenceNumber + "," + enable + "," + fl2int(roll) + "," + fl2int(pitch) + "," + fl2int(gaz) + "," + fl2int(yaw) + "\r";
  sequenceNumber++;
  #ifndef GAINSPAN
    ARsrl << command;
  #else
    sendwifi(command);
  #endif
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
  PCsrl << "calling LEDAnim" << endl;
  at = "AT*LED=";
  command = at + sequenceNumber + "," + animseq + ",1073741824," + duration + "\r\n";
  sequenceNumber++;
  #ifndef GAINSPAN
    ARsrl << command;
  #else
    sendwifi(command);
  #endif
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
  sendConfig("control:altitude_max","2000");
  sendConfig("control:euler_angle_max","0.35"); //between 0 and 0.52
  sendConfig("control:outdoor","FALSE");
  sendConfig("control:flight_without_shell","FALSE");
  send_control_commands();
  sendComwdg(100);
  sendFtrim();
  delay(50);
  drone_emergency_reset(); //clear emergency flag
  
  return 1;
}

int Command::drone_hover(int msec)
{
  int i = 0;
  while (i < msec) {
    #ifndef GAINSPAN
      ARsrl << makePcmd(1, 0, 0, 0, 0);
    #else
      sendwifi(makePcmd(1, 0, 0, 0, 0));
    #endif
    delay(100);
    i += 100;
  }
  return 1;
}

/*int Command::drone_move_up(int centimeter)
{
  int i = 0;
  while (i < centimeter) {
    ARsrl << makePcmd(1, 0, 0, 0.6, 0);
    delay(100);
    i += 10;
  }
  return 1;
}

int Command::drone_move_down(int centimeter)
{
  int i = 0;
  while (i < centimeter) {
    #ifndef GAINSPAN
      ARsrl << makePcmd(1, 0, 0, -0.5, 0);
    #else
      sendwifi(makePcmd(1, 0, 0, -0.5, 0));
    #endif
    delay(100);
    i += 10;
  }
  return 1;
}
*/
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
    if (debug) {
      PCsrl.write(ARsrl.read());
    }
  }
}

// volatile, since it is modified in an ISR.
volatile boolean inService = false;

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
}

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
