#ifndef Command_h
#define Command_h

#include "Arduino.h"


class Command {
  public:
    Command();
    String sendComwdg();
    String sendFtrim();
    String sendConfig(String option, String value);
    String sendRef(int bit9);
    String sendPcmd(int enable, int roll, int pitch, int gaz, int yaw);
    String sendAnim(int anim, int time);
    //void flightMode();
    //void checkStatus();
    //void checkSequenceNumber();
    void start_s2ip();
    void quit_s2ip();
  private:
    String at;
    String command;
};

#endif
