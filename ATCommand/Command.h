#ifndef Command_h
#define Command_h

#include "Arduino.h"
#include "Streaming.h"

#define ARsrl Serial1
#define PCsrl Serial

class Command {
  public:
    Command();
    String sendComwdg();
    String sendFtrim();
    String sendConfig(String option, String value);
    String sendRef(int bit9);
    String sendPcmd(int enable, int roll, int pitch, int gaz, int yaw);
    String sendAnim(int anim, int time);
	String LEDAnim(int duration);
    //void flightMode();
    //void checkStatus();
    //void checkSequenceNumber();
    int start_s2ip();
    void quit_s2ip();
	
	// return 1 if drone is initialized
	int init_drone();
	// return 1 if drone is hovering
	int drone_hover();
	
	
  private:
    String at;
    String command;
    
    int drone_is_hover;
};


#endif
