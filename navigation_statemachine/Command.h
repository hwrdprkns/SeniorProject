#ifndef Command_h
#define Command_h

#include "Arduino.h"
#include "Streaming.h"

#define ARsrl Serial
#define PCsrl Serial
#define WIFIsrl ARsrl

#define BAUD 115200
// adjust this base on how often you read your ring buffer
#define SERIAL_BUFFER_SIZE 1
// adjust this base on how often you receive message
#define SERIAL_INTERVAL_USEC 30000
#define COMWDG_INTERVAL_USEC 600000

typedef enum {
	TAKEOFF,
	LANDING,
	EMERGENCY_TOGGLE
} flying_status;

typedef enum {
	ARDRONE_ANIM_PHI_M30_DEG= 0,
	ARDRONE_ANIM_PHI_30_DEG,
	ARDRONE_ANIM_THETA_M30_DEG,
	ARDRONE_ANIM_THETA_30_DEG,
	ARDRONE_ANIM_THETA_20DEG_YAW_200DEG,
	ARDRONE_ANIM_THETA_20DEG_YAW_M200DEG,
	ARDRONE_ANIM_TURNAROUND,
	ARDRONE_ANIM_TURNAROUND_GODOWN,
	ARDRONE_ANIM_YAW_SHAKE,
	ARDRONE_ANIM_YAW_DANCE,
	ARDRONE_ANIM_PHI_DANCE,
	ARDRONE_ANIM_THETA_DANCE,
	ARDRONE_ANIM_VZ_DANCE,
	ARDRONE_ANIM_WAVE,
	ARDRONE_ANIM_PHI_THETA_MIXED,
	ARDRONE_ANIM_DOUBLE_PHI_THETA_MIXED,
	ARDRONE_NB_ANIM_MAYDAY
} anim_mayday_t;

class Command {
  public:
    Command();
    
    int start_wifi_connection(); //connect to the drone using wifi, set up connection protocols
    
    void sendComwdg(int msec); //send reset communication watchdog command repeatedly for a few secs
    void sendFtrim(); //send flat trim and tell the drone it's lying flat
    void sendConfig(String option, String value); //send configuration options
    void sendRef(flying_status fs); //send basic behavior commands (takeoff, landing etc.)
    void send_control_commands(); //send drone control mode commands
	  
    // clear emergency flag 
    void drone_emergency_toggle();
    
    String makeAnim(anim_mayday_t anim, int time); //send basic preset drone animation commands, not used
    void LEDAnim(int animseq, int duration); //send LED animation command
    
    // only used under serial connection, abandoned
    int start_s2ip();
    void quit_s2ip();
    
    // initialize the drone after the wifi connection is established, setup flight configuration
    int init_drone();
    
    int drone_hover(int msec); //make the drone hover for a set no. of secs
    int drone_takeoff(); //send takeoff command
    int drone_landing(); //send landing command
    
    void readARsrl();
    
    int s2ip_running;
    int drone_is_init;
    
    // Movement functions
    int moveForward(float distanceInMeters);
    int moveBackward(float distanceInMeters);
    int moveUp(float distanceInMeters);
    int moveDown(float distanceInMeters);
    int moveLeft(float distanceInMeters);
    int moveRight(float distanceInMeters);
    int moveRotate(int yawInDegrees); //degrees can be either positive (clockwise from top view) or negative
    
    // can only call after wifi's connection established and CID is given as 0
    void sendwifi(String s);
    
  private:
    String at;
    String command;
	
    // low level routine
    String makePcmd(int enable, float roll, float pitch, float gaz, float yaw); //send progressive commands that make the drone move (translate/rotate)
    long fl2int(float value); //convert float values into 32-bit integer values
};

struct ring_buffer
{
  unsigned char buffer[SERIAL_BUFFER_SIZE];
  volatile int head;
  volatile int tail;
};

// utilized by fl2int()
union resultint_
{
  long i;
  float f;
};

inline void store_char(unsigned char c, ring_buffer *buffer);
void SrlRead();
void read_rx_buf();

#endif
