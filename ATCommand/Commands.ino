#include "Command.h"
#include "Streaming.h"

Command com;
int sequenceNumber = 1;
int i = 1;
String atcmd = "";

void setup()
{
  Serial.begin(115200);
}

void loop()
{

  if(i == 1){
    com.start_s2ip();
    /*atcmd = com.sendComwdg();
    Serial.print(atcmd);
    atcmd = com.sendFtrim();
    Serial.print(atcmd);
    atcmd = com.sendConfig("general:navdata_demo","TRUE");
    Serial.print(atcmd);
    atcmd = com.sendConfig("control:altitude_max","2000");
    Serial.print(atcmd);
    atcmd = com.sendRef(1);
    Serial.print(atcmd);*/
    i++;
  }
  
  if (i < 10){
    /*atcmd = com.sendPcmd(1,0,0,0,0);
    Serial.print(atcmd);*/
    i++;
  }
  
  if (i == 10){
    com.quit_s2ip();
  } 
  //com.sendRef(0);
}
