#include "Arduino.h"
#include "Command.h"

extern int sequenceNumber;

Command::Command()
{
  at = "";
  command = "";
}

String Command::sendComwdg()
{
  at = "AT*COMWDG=";
  command = at + sequenceNumber + "\r";
  sequenceNumber++;
  return command;
}

String Command::sendFtrim()
{
  at = "AT*FTRIM=";
  command = at + sequenceNumber + "\r";
  sequenceNumber++;
  return command;
}

String Command::sendConfig(String option, String value)
{
  at = "AT*CONFIG=";
  command = at + sequenceNumber + ",\"" + option + "\",\"" + value + "\"\r";
  sequenceNumber++;
  return command;
}

String Command::sendRef(int bit9)
{
  at = "AT*REF=";
  if(bit9 == 1){
    command = at + sequenceNumber + ",290718208\r"; //takeoff
  }
  else if(bit9 == 0){
    command = at + sequenceNumber + ",290717696\r"; //landing
  }
  // emergency -> 290717952
  sequenceNumber++;
  return command;
}

String Command::sendPcmd(int enable, int roll, int pitch, int gaz, int yaw)
{
  at = "AT*PCMD=";
  command = at + sequenceNumber + "," + enable + "," + roll + "," + pitch + "," + gaz + "," + yaw + "\r";
  sequenceNumber++;
  return command;
}

String Command::sendAnim(int anim, int time)
{
  at = "AT*ANIM=";
  command = at + sequenceNumber + "," + anim + "," + time + "\r";
  sequenceNumber++;
  return command;
}

void Command::start_s2ip()
{
  while (Serial.available()){
  }
  
  Serial.print("\n");
  Serial.print("cd data/video/apps\n");
  Serial.print("./s2ip.arm -l\n"); 
}

void Command::quit_s2ip()
{
  Serial.write(03);
}
