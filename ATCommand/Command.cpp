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

int Command::start_wifi_connection(){
        WIFIsrl.begin(9600);
 
        WIFIsrl.print("+++");
        delay(1250);
        WIFIsrl.println("");
        WIFIsrl.println("AT&F");
        WIFIsrl.println("ATC1");
        WIFIsrl.println("AT+WM=0");
        //WIFIsrl.println("AT+WS");
        WIFIsrl.println("AT+NDHCP=1");
        WIFIsrl.println("AT+WA=ardrone_154516");
        WIFIsrl.println("AT+NAUTO=0,0,192.168.1.1,5556");
        //WIFIsrl.println("AT+NCUDP=192.168.1.1,5556");
        //WIFIsrl.println("AT+NSTAT=?");
        //WIFIsrl.println("AT+CID=?");
        WIFIsrl.print("ATA2\r");
        
        PCsrl.println("This is a test!");
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

String Command::sendRef(flying_status fs, int emergency)
{
  if (emergency == 1) {
    String emergency_ready = sendRef(LANDING);
    command = at + sequenceNumber + ",290717952\r\n";
    sequenceNumber++;
    return emergency_ready + command;
  }
}

/** Movement functions **/
int Command::moveForward(float distanceInMeters)
{
  float i = 0;
  String moveForward = makePcmd(1, 0, 1, 0, 0);
  sendPcmd(moveForward);
  delay(200);
  while (i < distanceInMeters) {
    String stayForward = makePcmd(1, 0, 0, 0, 0);
    sendPcmd(stayForward);
    delay(200);
    i += 0.2;
  }
  return 1;
  
  /*int pitchCalibrationFactor = -1;
  int delayFactor = 3;
  String moveForward = makePcmd(1,lastRoll,(lastPitch - pitchCalibrationFactor),lastGaz,lastYaw);
  sendPcmd(command);
  delay(delayFactor*distanceInMeters*50); //This is in millis, right?
  String hover = makePcmd(1,lastRoll,lastPitch,lastGaz,lastYaw);
  sendPcmd(hover);*/
}

int Command::moveRotate(float yawInDegrees)
{
  int i = 0;
  //String command = makePcmd(1, 0, 0, 0, 1);
  //sendPcmd(command);
  //delay(200);
  while (i < yawInDegrees) {
    //String stayRotate = makePcmd(1, 0, 0, 0, yawInDegrees/90);
    String stayRotate = makePcmd(1, 0, 0, 0, 0.17);
    sendPcmd(stayRotate);
    delay(150);
    i = i+8;
  }
 
  return 1;
  
  
  /*int rotateCalibrationFactor = 5;
  int delayFactor = 3;
  String moveRotate = makePcmd(1,lastRoll,lastPitch,lastGaz,(lastYaw + yawInDegrees*rotateCalibrationFactor));
  sendPcmd(moveRotate);
  delay(delayFactor*rotateCalibrationFactor*50);//Again... in millis?
  String hover = makePcmd(1,lastRoll,lastPitch,lastGaz,lastYaw);
  sendPcmd(hover);*/
}

String Command::makePcmd(int enable, float roll, float pitch, float gaz, float yaw)
{
  at = "AT*PCMD=";
  command = at + sequenceNumber + "," + enable + "," + fl2int(roll) + "," + fl2int(pitch) + "," + fl2int(gaz) + "," + fl2int(yaw) + "\r\n";
  sequenceNumber++;
  return command;
}

void Command::sendPcmd(String command)
{
  previousCommand = command;
  ARsrl << command;
}

String Command::makeAnim(anim_mayday_t anim, int time)
{
  at = "AT*ANIM=";
  command = at + sequenceNumber + "," + anim + "," + time + "\r\n";
  sequenceNumber++;
  return command;
}

String Command::LEDAnim(int animseq, int duration)
{
  //PCsrl << "calling LEDAnim" << endl;
  at = "AT*LED=";
  command = at + sequenceNumber + "," + animseq +",1073741824," + duration + "\r\n";
  sequenceNumber++;
  
  //PCsrl << command;
  
  return command;
}

//not used
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
  ARsrl << sendComwdg();
  //ARsrl << LEDAnim(2);
  ARsrl << sendConfig("general:navdata_demo","TRUE");
  ARsrl << sendConfig("control:altitude_max","2000");
  //ARsrl << sendConfig("control:outdoor","FALSE");
  //ARsrl << sendConfig("control:flight_without_shell","FALSE");
  ARsrl << sendFtrim();
  //ARsrl << sendRef(LANDING,1); //clear emergency flag
  emergency = 0;
  
  //do some checking??
  
  return 1;
}

int Command::drone_takeoff()
{
  ARsrl << sendRef(TAKEOFF);
  int i = 0;
  while (i < 50) {
    ARsrl << makePcmd(1, 0, 0, 1, 0);
    delay(100);
    i++;
  }
  return 1;
}

int Command::drone_hover(int msec)
{
  int i = 0;
  while (i < msec) {
    ARsrl << makePcmd(1, 0, 0, 0, 0);
    delay(100);
    i += 100;
  }
  return 1;
}

int Command::drone_landing()
{
  ARsrl << sendRef(LANDING);
  int i = 0;
  while (i < 50) {
    ARsrl << makePcmd(1, 0, 0, -1, 0);
    delay(100);
    i++;
  }
  return 1;
}

int Command::drone_move_up(int centimeter)
{
  int i = 0;
  while (i < centimeter) {
    ARsrl << makePcmd(1, 0, 0, 0.5, 0);
    delay(100);
    i = i+10;
  }
  return 1;
}

int Command::drone_move_down(int centimeter)
{
  int i = 0;
  while (i < centimeter) {
    ARsrl << makePcmd(1, 0, 0, -0.5, 0);
    delay(100);
    i = i+10;
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
