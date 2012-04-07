#ifndef Command_h
#define Command_h

#include "Arduino.h"
#include "Streaming.h"

#define ARsrl Serial1
#define PCsrl Serial

typedef enum {
	TAKEOFF,
	LANDING
} flying_status;
	
	
class Command {
  public:
    Command();
    String sendComwdg();
    String sendFtrim();
    String sendConfig(String option, String value);
    String sendRef(flying_status fs);
	
	// clear emergency flag && fs set to LANDING if emergency == 1, 
	String sendRef(flying_status fs, int emergency);
	
   
	String sendPcmd(int enable, float roll, float pitch, float gaz, float yaw);
    String sendAnim(int anim, int time);
	String LEDAnim(int duration);
    //void flightMode();
    //void checkStatus();
    //void checkSequenceNumber();
    int start_s2ip();
    void quit_s2ip();
	
	// return 1 if drone is initialized
	int init_drone();
	
	int drone_takeoff();
	
	// return 1 if drone is hovering
	int drone_hover();
	int drone_landing();
	
	void readARsrl();
	
	int s2ip_running;
	int drone_is_hover;
	int drone_is_init;
	int emergency;

	
  private:
    String at;
    String command;
	
	int fl2int(float value);
	String sendPcmd(int enable, int roll, int pitch, int gaz, int yaw);
};



#endif
