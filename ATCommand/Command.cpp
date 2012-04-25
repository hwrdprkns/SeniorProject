
#ifndef GAINSPAN
#define GAINSPAN

#include "Command.h"


extern int sequenceNumber;
extern int debug;

ring_buffer rx_buf= {{0}, 0, 0};
resultint_ resultint;

const char ATF[] PROGMEM = "AT&F";
const char ATMAC[] PROGMEM = "AT+NMAC=00:1d:c9:10:39:6f\r";
const char ATWM[] PROGMEM = "AT+WM=0";
const char ATNDHCP[] PROGMEM = "AT+NDHCP=1";
const char ATNCUDP[] PROGMEM = "AT+NCUDP=192.168.1.1,5556";

const char ATCONFIG[] PROGMEM = "AT*CONFIG=";
const char ATCOMWDG[] PROGMEM = "AT*COMWDG=";
const char ATPCMD[] PROGMEM = "AT*CTRL="



const char *wirelessTable[] PROGMEM =	   // change "string_table" name to suit
{   
  ATF,
  ATMAC,
  ATWM,
  ATNDHCP,
  ATNCUDP}; 


Command::Command()
{
  command = "";
  s2ip_running = 0;
  drone_is_init = 0;
  drone_is_hover = 0;
  emergency = 0;
}


void Command::sendwifi(String s) {
  
    WIFIsrl.write(27); //esc
    WIFIsrl.print("S0"); //choose connection CID 0
    WIFIsrl.print(s);
    WIFIsrl.write(27);
    WIFIsrl.print("E");
    
       
    if(debug) PCsrl.println(s);
    
    WIFIsrl.println(memoryTest()); 
}



int Command::start_wifi_connection()
{
  WIFIsrl.begin(9600);
 
  WIFIsrl.println("");
  WIFIsrl.println(bufferString(0));
  //WIFIsrl.println("ATE0"); //turn off echo
  WIFIsrl.print(bufferString(1)); //set MAC address
  WIFIsrl.println(bufferString(2));
  
  WIFIsrl.println(bufferString(3));
  
  /* drone's network profile, change if needed*/
  WIFIsrl.println("AT+WA=ardrone_279440");
  WIFIsrl.println(bufferString(4));
  readARsrl();

  delay(3000); //need 3 seconds for connection to establish
  return 0;
}  

char* Command::bufferString(int which){
  strcpy_P(buffer, (char*)pgm_read_word(&(wirelessTable[which])));
  return buffer;
}
  
  
  
String Command::makeComwdg()
{
  String s = "AT*COMWDG=";
  command = s + getSequenceNumber() + "\r\n";
  return command;
}

void Command::sendComwdg_t(int msec)
{
  for (int i = 0; i < msec; i+=20) {
    sendwifi(makeComwdg());
    delay(20);
  }
}

void Command::sendFtrim()
{
  String at = "AT*FTRIM=";
  command = at + getSequenceNumber() + "\r\n";
  sendwifi(command);
}

void Command::sendConfig(String option, String value)
{
  String at = "AT*CONFIG=";
  command = at + getSequenceNumber() + ",\"" + option + "\",\"" + value + "\"\r\n";
  sendwifi(command);
}

void Command::sendRef(flying_status fs)
{
  String at = "AT*REF=";
  if(fs == TAKEOFF){
    command = at + getSequenceNumber() + ",290718208\r\n"; //takeoff
  }
  else if(fs == LANDING){
    command = at + getSequenceNumber() + ",290717696\r\n"; //landing
  } else if (fs == EMERGENCY_TOGGLE){
    command = at + getSequenceNumber() + ",290717952\r\n"; //landing
  }
   
  // emergency -> 290717952
  sendwifi(command);
}

void Command::send_control_commands(){
    String at = "AT*CTRL=";
    sendwifi(at+getSequenceNumber()+",4,0\r\n");
    sendwifi(at+getSequenceNumber()+",0,0\r\n");
    sendwifi(at+getSequenceNumber()+",4,0\r\n");
}

void Command::drone_emergency_reset()
{
  String at = "AT*REF=";
  command = at + getSequenceNumber() + ",290717952\r\n";
  sendwifi(command);
}

/** Movement functions **/
int Command::moveForward(float distanceInMeters)
{
  float i = 0;
  String moveForward = makePcmd(1, 0, -.855, 0, 0);
  delay(1000*distanceInMeters);
  sendPcmd(moveForward);
  return 1;
}

int Command::moveRotate(float yawInDegrees)
{
  int i = 0;
  while (i < yawInDegrees) {
    String stayRotate = makePcmd(1, 0, 0, 0, 0.17);
    sendPcmd(stayRotate);
    delay(150);
    i += 8;
  }
  return 1;
}

String Command::makePcmd(int enable, float roll, float pitch, float gaz, float yaw)
{
  String at = "AT*PCMD=";
  command = at + getSequenceNumber() + "," + enable + "," + fl2int(roll) + "," + fl2int(pitch) + "," + fl2int(gaz) + "," + fl2int(yaw) + "\r";
  return command;
}

void Command::sendPcmd(String command)
{
  previousCommand = command;
  sendwifi(command);
}

void Command::sendPcmd(int enable, float roll, float pitch, float gaz, float yaw)
{
  String at = "AT*PCMD=";
  command = at + getSequenceNumber() + "," + enable + "," + fl2int(roll) + "," + fl2int(pitch) + "," + fl2int(gaz) + "," + fl2int(yaw) + "\r";
  sendwifi(command);
}

String Command::makeAnim(anim_mayday_t anim, int time)
{
  String at = "AT*ANIM=";
  command = at + getSequenceNumber() + "," + anim + "," + time + "\r\n";
  return command;
}

void Command::doLEDAnim(int animseq, int duration)
{
  PCsrl << "calling LEDAnim" << endl;
  String at = "AT*LED=";
  command = at + getSequenceNumber() + "," + animseq + ",1073741824," + duration + "\r\n";
  sendwifi(command);
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
  sendConfig("general:navdata_demo","TRUE");
  sendConfig("control:altitude_max","2000");
  sendConfig("control:euler_angle_max","0.35");
  sendConfig("control:outdoor","FALSE");
  sendConfig("control:flight_without_shell","FALSE");
  send_control_commands();
  sendComwdg_t(90);
  sendFtrim();
  
  drone_emergency_reset(); //clear emergency flag
  
  return 1;
}

int Command::drone_takeoff()
{
  sendRef(TAKEOFF);
  int i = 0;
  return 1;
}

int Command::drone_hover(int msec)
{
  int i = 0;
  while (i < msec) {
    sendwifi(makePcmd(1, 0, 0, 0, 0));
    delay(100);
    i += 100;
  }
  return 1;
}

int Command::drone_landing()
{
  sendRef(LANDING);
  return 1;
}

int Command::drone_move_up(int centimeter)
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
    sendwifi(makePcmd(1, 0, 0, -0.5, 0));
    delay(100);
    i += 10;
  }
  return 1;
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
    if (debug) {
      PCsrl.write(ARsrl.read());
    }
  }
}


//Memory test code from : http://www.faludi.com/2007/04/18/arduino-available-memory-test/
int Command::memoryTest() {
  int byteCounter = 0; // initialize a counter
  byte *byteArray; // create a pointer to a byte array
  // More on pointers here: http://en.wikipedia.org/wiki/Pointer#C_pointers

  // use the malloc function to repeatedly attempt
  // allocating a certain number of bytes to memory
  // More on malloc here: http://en.wikipedia.org/wiki/Malloc
  while ( (byteArray = (byte*) malloc (byteCounter * sizeof(byte))) != NULL ) {
    byteCounter++; // if allocation was successful, then up the count for the next try
    free(byteArray); // free memory after allocating it
  }

  free(byteArray); // also free memory after the function finishes
  return byteCounter; // send back the highest number of bytes successfully allocated
}

int Command::getSequenceNumber(){
  return sequenceNumber++;
}

// Volatile, since it is modified in an ISR.
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
